#!/usr/bin/env python3
"""Host checks using this repository's actual CMSIS/HAL/USB headers.
Requires GCC and Python 3. No board or Keil compiler is emulated.
"""
import argparse
import csv
import importlib.util
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sanitize", action="store_true")
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    include_names = ["Core/Inc", "Drivers/UART", "USB_DEVICE/App", "USB_DEVICE/Target", "Drivers/STM32F1xx_HAL_Driver/Inc",
                     "Drivers/STM32F1xx_HAL_Driver/Inc/Legacy", "Middlewares/ST/STM32_USB_Device_Library/Core/Inc",
                     "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc", "Drivers/CMSIS/Device/ST/STM32F1xx/Include",
                     "Drivers/CMSIS/Include", "tests"]
    flags = ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wno-int-to-pointer-cast", "-Wno-pointer-to-int-cast", "-DUSE_HAL_DRIVER", "-DSTM32F103xB"]
    flags += ["-I" + str(root / name) for name in include_names]
    changed = ["Core/Src/main.c", "Core/Src/rng9.c", "Core/Src/ADC.c", "Core/Src/Timer.c", "Drivers/UART/UART.c", "Core/Src/usb_cdc_app.c",
               "Core/Src/stm32f1xx_hal_msp.c", "Core/Src/stm32f1xx_it.c", "USB_DEVICE/App/usbd_cdc_if.c"]
    subprocess.run(["gcc", *flags, "-fsyntax-only", *[str(root / path) for path in changed]], check=True)
    print("PASS C99 syntax: merged sources with actual repository headers", flush=True)
    project = root / "MDK-ARM" / "he_thong_nhung.uvprojx"
    tree = ET.parse(project)
    paths = [node.text for node in tree.findall(".//Files/File/FilePath")]
    assert len(paths) == len(set(paths)), "Duplicate Keil source entries"
    for name in paths:
        assert (project.parent / name.replace("\\", "/")).is_file(), name
    for name in ("../Core/Src/ADC.c", "../Core/Src/rng9.c", "../Core/Src/Timer.c", "../Drivers/UART/UART.c", "../Core/Src/usb_cdc_app.c"):
        assert name in paths, name
    assert "__HAL_AFIO_REMAP_SWJ_DISABLE()" not in (root / "Core/Src/stm32f1xx_hal_msp.c").read_text()
    assert "VP_SYS_VS_ND" not in (root / "he_thong_nhung.ioc").read_text()
    print("PASS project: all Keil source paths, unique entries, DMA modules and SWD config", flush=True)
    spec = importlib.util.spec_from_file_location("receiver", root / "PC" / "receive_adc.py")
    receiver = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(receiver)
    sys.path.insert(0, str(root / "PC/TRNG_Control"))
    from trng.controller import Controller
    from trng.protocol import Config, WireFramer, parse_line
    from trng.storage import SessionStore
    if args.sanitize:
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    environment = os.environ.copy()
    environment["ASAN_OPTIONS"] = "detect_leaks=0"
    with tempfile.TemporaryDirectory(prefix="cdc_adc_merge_test_") as directory:
        build = Path(directory)
        executable = build / "test_merge"
        subprocess.run(["gcc", *flags, "-include", str(root / "tests/mcu_mock.h"),
                        str(root / "tests/test_merge.c"), str(root / "Core/Src/rng9.c"), str(root / "Core/Src/usb_cdc_app.c"),
                        str(root / "USB_DEVICE/App/usbd_cdc_if.c"), str(root / "Drivers/UART/UART.c"),
                        str(root / "Core/Src/stm32f1xx_it.c"), "-lm", "-o", str(executable)], check=True)
        uart_test = build / "test_uart"
        subprocess.run(["gcc", *flags, "-include", str(root / "tests/mcu_mock.h"),
                        str(root / "tests/test_uart.c"), "-o", str(uart_test)], check=True)
        subprocess.run([str(uart_test)], check=True, env=environment)
        full, small, stream, fast, uart_default, uart_max, rng9_wire, rng9_stream_wire = (build / name for name in ("full.txt", "small.txt", "stream.txt", "fast.txt", "uart_default.bin", "uart_max.bin", "rng9.bin", "rng9_stream.bin"))
        subprocess.run([str(executable), str(full), str(small), str(stream), str(fast), str(uart_default), str(uart_max), str(rng9_wire), str(rng9_stream_wire)], check=True, env=environment)
        # Real firmware/main UART/CDC binary output must pass the Python
        # RNG9 parser/controller, stateful algorithm and .bin serializer.
        import struct
        import zlib
        from trng.rng9 import Sine9, pack_words
        completed = []
        controller = Controller(lambda data: None,
                                lambda kind, value: completed.append(value) if kind == "capture_complete" else None)
        controller.rng9_supported = True
        controller.configure(Config(1000, 512, 32, "rng9"), start=True)
        framer = WireFramer()
        payload = rng9_wire.read_bytes()
        assert b"BEGINR," in payload and b"\xA5\x5A\xC3\x3C\x01\x03" in payload
        for pos in range(0, len(payload), 7):
            for kind, value in framer.feed(payload[pos:pos+7]):
                controller.accept(*(parse_line(value) if kind == "line" else (kind, value)))
        assert controller.state == "idle" and len(completed) == 1
        expected_engine = Sine9()
        expected = [expected_engine.next((i * 29) & 4095) for i in range(512)]
        assert completed[0]["samples"] == expected
        store = SessionStore(build / "rng9_sessions", "MCU_MODEL", 115200)
        try:
            output = store.rng_capture(completed[0])
            assert output.read_bytes() == pack_words(expected)
            assert output.stat().st_size == 576
        finally:
            store.close()
        print("PASS C firmware -> RNG9 words -> Python CRC -> RNG9 .bin: 512 samples / 4608 bits", flush=True)
        # Long RNG9 stream checks persistence of SINE9 phase across approximately 1,042
        # binary frames and hundreds of circular DMA blocks.
        completed = []
        ctrl = Controller(lambda data: None,
                          lambda kind, value: completed.append(value) if kind == "capture_complete" else None)
        ctrl.rng9_supported = True
        ctrl.configure(Config(3000, 50000, 32, "rng9"), start=True)
        ctrl.accept("config_ok", None)
        ctrl.accept("start_ok", None)
        framer = WireFramer()
        payload = rng9_stream_wire.read_bytes()
        for pos in range(0, len(payload), 17):
            for kind, value in framer.feed(payload[pos:pos + 17]):
                ctrl.accept(*(parse_line(value) if kind == "line" else (kind, value)))
        assert ctrl.state == "idle" and len(completed) == 1
        assert completed[0]["wire_format"] == "rng9"
        reference = Sine9()
        expected = [reference.next((i * 29) & 4095) for i in range(50000)]
        assert completed[0]["samples"] == expected
        store = SessionStore(build / "rng9_stream_sessions", "MCU_MODEL", 115200)
        try:
            output = store.rng_capture(completed[0])
            assert output.read_bytes() == pack_words(expected)
            assert output.stat().st_size == 56250
        finally:
            store.close()
        print("PASS 50,000 RNG9 samples across circular DMA -> Python -> 56,250-byte .bin", flush=True)
        for path, count, rate in ((full, 512, 1000), (small, 256, 2000), (stream, 50000, 1000), (fast, 1000000, 100000),
                                  (uart_default, 10000, 1000), (uart_max, 50000, 3000)):
            parser = receiver.CaptureParser(count)
            payload = path.read_bytes()
            link = "UART" if path in (uart_default, uart_max) else "USB"
            for i in range(0, len(payload), 7):
                parser.feed(payload[i:i + 7])
            assert parser.complete and parser.samples == [(i * 29) & 4095 for i in range(count)]
            print(f"PASS C ({link}) -> PC: {count}/{count}, requested {rate} Hz (modeled), CRC32={parser.crc:08X}", flush=True)
            completed = []
            controller = Controller(lambda data: None,
                                    lambda kind, value: completed.append(value) if kind == "capture_complete" else None)
            controller.configure(Config(adc_rate=rate, samples=count), start=True)
            framer = WireFramer()
            for i in range(0, len(payload), 7):
                for kind, value in framer.feed(payload[i:i + 7]):
                    controller.accept(*(parse_line(value) if kind == "line" else (kind, value)))
            assert controller.state == "idle" and len(completed) == 1
            assert completed[0]["samples"] == parser.samples
            store = SessionStore(build / "gui_sessions", "MODELED_C_TRANSPORT", 115200)
            try:
                path = store.capture(completed[0])
                with path.open(encoding="utf-8-sig", newline="") as file:
                    assert sum(1 for _ in csv.DictReader(file)) == count
            finally:
                store.close()
            print(f"PASS C ({link}) -> app controller -> validated CSV: {count} samples", flush=True)
    subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", str(root / "tests"), "-p", "test_receiver.py", "-v"], check=True)
    subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py", "-v"],
                   cwd=root / "PC/TRNG_Control", check=True)


if __name__ == "__main__":
    main()
