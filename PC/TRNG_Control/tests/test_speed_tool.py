"""Tests real-port benchmark logic with a strict simulated serial device."""
import importlib.util
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("speed_test", ROOT / "test_rng9_speed.py")
speed = importlib.util.module_from_spec(spec)
spec.loader.exec_module(speed)

from trng.transport import SimulatedDevice
from trng.protocol import WireFramer


class DeviceAdapter:
    def __init__(self):
        self.dev = SimulatedDevice()

    def reset_input_buffer(self):
        self.dev.output.clear()

    def write(self, data):
        n = self.dev.write(data)
        if b"START\n" in data:
            self.dev.started -= 50
        return n

    def read(self, count):
        return self.dev.read(count)


class SpeedToolTests(unittest.TestCase):
    def test_measure_simulated_crc_fifo_and_info(self):
        device = DeviceAdapter()
        device.write(b"GET_INFO\n")
        # Simulated INFO intentionally predates PACK9 capability field.
        info = speed.read_until(device, WireFramer(), lambda kind, value: value if kind == "info" else None,
                                time.monotonic() + 2)
        self.assertIn("simulator", info)
        result = speed.measure_one(device, 3000, 500)
        self.assertEqual(result["samples"], 500)
        self.assertEqual(result["rate_hz"], 3000)
        self.assertEqual(len(result["crc32"]), 8)

    def test_batched_start_ack_and_begin_are_not_lost(self):
        class BytesDevice:
            def __init__(self):
                self.bytes = [b"START_OK\nBEGINR,1,500,3000000\n"]
            def read(self, count):
                return self.bytes.pop(0) if self.bytes else b""
        mock = BytesDevice()
        frame = WireFramer()
        self.assertTrue(speed.read_until(mock, frame,
                                         lambda kind, value: True if kind == "start_ok" else None,
                                         time.monotonic() + 1))
        self.assertEqual(speed.read_until(mock, frame,
                                          lambda kind, value: value if kind == "capture_begin_rng" else None,
                                          time.monotonic() + 1), (1, 500, 3000000))


if __name__ == "__main__":
    unittest.main()
