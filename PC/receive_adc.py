#!/usr/bin/env python3
"""Configure STM32 over USB CDC and receive a complete PA0 ADC capture.
Only commit CSV after sample count, session, order and CRC all validate.
Python 3.9+. Install pyserial; matplotlib is optional with --plot.
"""
import argparse
import csv
import os
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "TRNG_Control"))
from trng.protocol import WireFramer, CaptureAssembler, Config, MAX_ADC_RATE_HZ, parse_line


class CaptureError(ValueError):
    pass


class CaptureParser:
    """Shared binary/text decoder; commit only a fully validated capture."""
    def __init__(self, expected=512):
        self.expected = expected
        self.session = None
        self.total = self.rate_millihz = self.crc = 0
        self.samples = []
        self.complete = False
        self.framer = WireFramer(limit=127)
        self.capture = None

    def feed(self, chunk):
        if self.complete and chunk:
            raise CaptureError("Unexpected bytes after END")
        for kind, value in self.framer.feed(chunk):
            if self.complete:
                raise CaptureError("Extra frames after END")
            if kind == "line":
                kind, value = parse_line(value)
            self._accept(kind, value)
        if self.complete and self.framer.buffer:
            raise CaptureError("Extra bytes after END")

    def _accept(self, kind, value):
        try:
            if kind in {"parse_error", "error", "capture_error"}:
                raise CaptureError(str(value))
            if kind in {"config_ok", "start_ok", "stop_ok", "info"}:
                if self.capture is not None:
                    raise CaptureError("Unexpected control response inside ADC stream")
                return
            if kind in {"capture_begin", "capture_begin_binary"}:
                if self.capture is not None:
                    raise CaptureError("Second BEGIN before END")
                session, total, rate = value
                if self.expected is not None and total != self.expected:
                    raise CaptureError(f"Expected {self.expected} samples; STM32 announced {total}")
                self.capture = CaptureAssembler(*value, wire_format="binary" if kind == "capture_begin_binary" else "text")
                self.session, self.total, self.rate_millihz = session, total, rate
                self.samples = self.capture.samples
            elif kind in {"capture_data", "capture_block"}:
                if self.capture is None:
                    raise CaptureError("ADC data without BEGIN")
                if kind == "capture_block":
                    self.capture.add_block(*value)
                else:
                    self.capture.add(*value)
                self.crc = self.capture.crc
            elif kind == "capture_metrics":
                if self.capture is None or self.capture.session != value[0] or self.capture.metrics is not None:
                    raise CaptureError("Invalid/duplicate metrics")
                self.capture.metrics = dict(zip(("fifo_peak", "copy_max_cycles", "pack_max_cycles", "capture_ms", "core_hz"), value[1:]))
            elif kind == "capture_end":
                if self.capture is None:
                    raise CaptureError("END without BEGIN")
                self.capture.finish(*value)
                self.complete = True
            else:
                raise CaptureError(f"Unknown or misplaced frame: {kind}, {value!r}")
        except ValueError as exc:
            raise CaptureError(str(exc)) from exc


def save_csv(parser, path, vref=3.3):
    if not parser.complete:
        raise CaptureError("Refusing to save an incomplete capture")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", newline="", encoding="utf-8", delete=False,
                                         dir=path.parent, prefix=path.name + ".", suffix=".part") as file:
            temporary = Path(file.name)
            writer = csv.writer(file)
            writer.writerow(["session", "index", "time_s", "adc", "voltage_V"])
            for index, value in enumerate(parser.samples):
                writer.writerow([parser.session, index, f"{index * 1000 / parser.rate_millihz:.9f}",
                                 value, f"{value * vref / 4095:.6f}"])
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path.resolve()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="Windows example: COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--samples", "--expect", dest="samples", type=int, default=512)
    ap.add_argument("--rate", type=int, default=1000, help=f"Requested ADC samples per second, 1..{MAX_ADC_RATE_HZ}")
    ap.add_argument("--size", type=int, choices=(32, 64, 128), default=32,
                    help="Retained configuration field; raw ADC mode does not generate random output")
    ap.add_argument("--timeout", type=float, default=None, help="Override total capture+transfer timeout, seconds")
    ap.add_argument("--out", default="samples.csv")
    ap.add_argument("--vref", type=float, default=3.3, help="Measured VDDA in volts for CSV conversion")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()
    if args.timeout is None:
        args.timeout = max(15.0, args.samples / max(args.rate, 1) + 10.0)
    if args.timeout <= 0 or args.vref <= 0 or args.baud <= 0:
        ap.error("timeout, vref and baud must be positive")
    if not 2 <= args.samples <= 1000000 or args.samples % 2 or not 1 <= args.rate <= MAX_ADC_RATE_HZ:
        ap.error(f"samples must be even, 2..1000000; rate must be 1..{MAX_ADC_RATE_HZ} Hz")
    try:
        Config(args.rate, args.samples, args.size).validate()
    except ValueError as exc:
        ap.error(str(exc))
    try:
        import serial
    except ImportError:
        ap.error("Install pyserial: py -m pip install pyserial")
    parser = CaptureParser(args.samples)
    try:
        with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=2,
                           bytesize=8, parity="N", stopbits=1,
                           xonxoff=False, rtscts=False, dsrdtr=False) as port:
            port.reset_input_buffer()
            configuration = (f"CMD=CONFIG\nADC_RATE={args.rate}\nSAMPLE={args.samples}\nSIZE={args.size}\n").encode("ascii")
            port.write(configuration)
            port.flush()
            config_deadline = time.monotonic() + 5.0
            response = bytearray()
            while True:
                if time.monotonic() >= config_deadline:
                    raise CaptureError("Timeout waiting for CONFIG_OK; check firmware, COM and USB connection")
                byte = port.read(1)
                if not byte:
                    continue
                response.extend(byte)
                if len(response) > 128:
                    raise CaptureError("Oversized configuration response")
                if byte == b"\n":
                    text = response.rstrip(b"\r\n").decode("ascii")
                    response.clear()
                    if text == "CONFIG_OK":
                        break
                    if text.startswith(("ERROR", "ERR=")):
                        raise CaptureError(text)
                    if text:
                        raise CaptureError(f"Unexpected configuration response: {text!r}")
            port.write(b"START\n")
            port.flush()
            print(f"CONFIG_OK; START on {args.port}; {args.samples} samples at {args.rate} Hz...")
            deadline = time.monotonic() + args.timeout
            while not parser.complete:
                if time.monotonic() >= deadline:
                    raise CaptureError(f"Timeout: received {len(parser.samples)}/{parser.total or args.samples} samples")
                chunk = port.read(min(max(port.in_waiting, 1), 512))
                if chunk:
                    parser.feed(chunk)
        path = save_csv(parser, args.out, args.vref)
    except (serial.SerialException, OSError, UnicodeError, CaptureError) as exc:
        raise SystemExit(f"FAIL: {exc}. No new validated CSV was saved.") from exc
    print(f"PASS: {len(parser.samples)}/{parser.total} samples; indices 0..{parser.total - 1}; "
          f"CRC32={parser.crc:08X}; rate={parser.rate_millihz / 1000:g} Hz")
    print(f"CSV: {path}")
    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            raise SystemExit("CSV saved. Install matplotlib: py -m pip install matplotlib")
        times = [i * 1000 / parser.rate_millihz for i in range(parser.total)]
        plt.plot(times, parser.samples, linewidth=1)
        plt.xlabel("Time relative to first sample (s)")
        plt.ylabel("ADC (0..4095)")
        plt.title(f"{parser.total} samples, {parser.rate_millihz / 1000:g} Hz, CRC OK")
        plt.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
