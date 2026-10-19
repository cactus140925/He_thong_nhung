#!/usr/bin/env python3
"""Hardware benchmark for SINE9 over STM32 USB CDC, retaining CRC/fifo checks.

Run after flashing the optimized firmware, with the GUI disconnected:
    python test_rng9_speed.py --port COM10 --rates 3000 5000 10000 --samples 50000

This does not claim any speed/entropy property without the actual board.
"""
import argparse
from collections import deque
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from trng.protocol import Config, WireFramer, parse_line, CaptureAssembler, FIFO_SAMPLES


class RateTestError(RuntimeError):
    pass


def read_until(device, frame, predicate, deadline):
    # Reuse the framer's pending decoded events between consecutive ACK stages.
    # Otherwise START_OK and BEGINR arriving in the same OS read would lose BEGINR.
    if not hasattr(frame, "pending_events"):
        frame.pending_events = deque()
    while time.monotonic() < deadline:
        if not frame.pending_events:
            incoming = device.read(4096)
            if not incoming:
                continue
            frame.pending_events.extend(frame.feed(incoming))
        while frame.pending_events:
            kind, value = frame.pending_events.popleft()
            event = parse_line(value) if kind == "line" else (kind, value)
            result = predicate(*event)
            if result is not None:
                return result
    raise RateTestError("Timeout chờ STM32; kiểm tra USB CDC và RX/TX")


def require_ack(device, frame, expected, seconds=5):
    def accept(kind, value):
        if kind == "error" or kind == "capture_error":
            raise RateTestError(f"Firmware báo lỗi: {value}")
        return True if kind == expected else None
    read_until(device, frame, accept, time.monotonic() + seconds)


def send_and_ack(device, frame, payload, expected):
    device.write(payload)
    require_ack(device, frame, expected)


def measure_one(device, rate, samples):
    Config(rate, samples, 32, "rng9").validate()
    framer = WireFramer()
    device.reset_input_buffer()
    send_and_ack(device, framer, Config(rate, samples, 32, "rng9").wire(), "config_ok")
    send_and_ack(device, framer, b"START\n", "start_ok")
    state = {"capture": None, "metrics": None}
    start = time.monotonic()
    # Allow sampling period, plus bounded transfer/drain time; abort on overload.
    deadline = start + max(10.0, 3 * samples / rate + 8.0)

    def accept(kind, value):
        if kind in ("capture_begin_rng", "capture_begin_binary"):
            if kind != "capture_begin_rng" or state["capture"] is not None:
                raise RateTestError("Sai chế độ hoặc BEGIN trùng")
            sid, count, mhz = value
            if count != samples or abs(mhz - rate * 1000) > rate * 10:
                raise RateTestError("Số mẫu/tần số timer trả về không khớp")
            state["capture"] = CaptureAssembler(sid, count, mhz, "rng9")
        elif kind == "capture_block_rng":
            if state["capture"] is None:
                raise RateTestError("Nhận DATA trước BEGINR")
            state["capture"].add_block(*value)
        elif kind in ("capture_block", "capture_data"):
            raise RateTestError("RNG9 nhận nhầm dữ liệu ADC RAW")
        elif kind == "capture_metrics":
            state["metrics"] = value
        elif kind == "capture_end":
            if state["capture"] is None:
                raise RateTestError("END trước BEGIN")
            record = state["capture"].finish(*value)
            if state["metrics"] is None:
                raise RateTestError("Không có thống kê MTR của firmware")
            session, fifo_peak, copy_cycles, pack_cycles, capture_ms, core_hz = state["metrics"]
            if session != record["device_session"]:
                raise RateTestError("MTR sai phiên")
            if fifo_peak > int(FIFO_SAMPLES * 0.8):
                raise RateTestError(f"FIFO peak quá cao ({fifo_peak}/{FIFO_SAMPLES})")
            expected_ms = samples * 1000 / rate
            if not 0 < capture_ms <= expected_ms * 1.25 + 5:
                raise RateTestError(f"Thời gian lấy mẫu {capture_ms} ms không khớp dự kiến {expected_ms:.1f} ms")
            # Successful CRC32 and each individual sample index are checked above.
            return {"rate_hz": rate, "samples": samples, "fifo_peak": fifo_peak,
                    "crc32": record["crc32"], "capture_ms": capture_ms,
                    "copy_cycles": copy_cycles, "pack_cycles": pack_cycles,
                    "core_hz": core_hz, "wall_ms": (time.monotonic() - start) * 1000}
        elif kind in ("parse_error", "error", "capture_error"):
            raise RateTestError(f"Khung lỗi/firmware lỗi: {value}")
        return None

    try:
        return read_until(device, framer, accept, deadline)
    except Exception:
        # STOP is best effort; disconnect/reconnect before resuming from a timeout.
        try:
            device.write(b"STOP\n")
        except Exception:
            pass
        raise


def main(argv=None):
    ap = argparse.ArgumentParser(description="Measure RNG9 hardware speed with exact CRC and FIFO integrity")
    ap.add_argument("--port", required=True, help="STM32 USB CDC port, e.g. COM10")
    ap.add_argument("--rates", type=int, nargs="+", default=[3000, 5000, 10000])
    ap.add_argument("--samples", type=int, default=50000)
    ap.add_argument("--repeats", type=int, default=2)
    args = ap.parse_args(argv)
    if args.repeats < 1 or any(r < 1 or r > 100000 for r in args.rates):
        ap.error("--repeats >= 1, 1 <= rate <= 100000")
    for rate in args.rates:
        Config(rate, args.samples, 32, "rng9").validate()
    try:
        import serial
    except ImportError:
        ap.error("Missing pyserial; run INSTALL.bat inside PC/TRNG_Control")
    print("Chỉ sử dụng khi đã đóng/ngắt Data Logger và nạp firmware PACK9=1.")
    with serial.Serial(args.port, 115200, timeout=.1, write_timeout=2) as dev:
        framer = WireFramer()
        dev.reset_input_buffer()
        dev.write(b"GET_INFO\n")
        info = read_until(dev, framer,
                          lambda kind, value: value if kind == "info" else None,
                          time.monotonic() + 5)
        print("INFO:", info)
        if "RNG9=1" not in info or "PACK9=1" not in info or "LINK=USB" not in info:
            raise RateTestError("Cần USB CDC + firmware RNG9 packed mới (PACK9=1)")
        if any(r > int(next((entry[3:] for entry in info.split(";") if entry.startswith("HZ=")), "0")) for r in args.rates):
            raise RateTestError("Tốc độ yêu cầu vượt trần của firmware")
        best = None
        for rate in sorted(set(args.rates)):
            print(f"Đo {rate:,} Hz / {args.samples:,} mẫu x {args.repeats} lượt", flush=True)
            for attempt in range(1, args.repeats + 1):
                try:
                    record = measure_one(dev, rate, args.samples)
                    time.sleep(.2)  # allow firmware END TX drain before next CONFIG
                    print(f"  #{attempt}: PASS crc={record['crc32']} FIFO={record['fifo_peak']}/{FIFO_SAMPLES} "
                          f"ADC={record['capture_ms']} ms PACKmax={record['pack_cycles']} cycles", flush=True)
                except Exception as exc:
                    print(f"  FAIL: {exc}. DỪNG tăng tốc; ngắt kết nối rồi thử mức thấp hơn.", file=sys.stderr)
                    return 1
            best = rate
        print(f"Mức cao nhất PASS trong lần đo này: {best:,} Hz. Không phải chứng nhận tốc độ lâu dài/entropy.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RateTestError, OSError) as exc:
        print("LỖI:", exc, file=sys.stderr)
        sys.exit(2)
