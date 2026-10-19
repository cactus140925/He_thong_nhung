"""Regression: fragmented USB reads must not create one GUI event per byte."""
import csv
from pathlib import Path
import queue
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zlib

from trng.controller import Controller
from trng.protocol import Config, WireFramer, encode_binary_block, parse_line
from trng.storage import SessionStore
from trng.transport import SerialWorker


def wire_capture(count=50000, binary=False):
    values = [(i * 29) & 4095 for i in range(count)]
    crc = zlib.crc32(b"".join(struct.pack("<H", v) for v in values))
    if binary:
        payload = f"CONFIG_OK\nSTART_OK\nBEGINB,4,{count},40000000\n".encode()
        payload += b"".join(encode_binary_block(4, i, values[i:i+21]) for i in range(0, count, 21))
        payload += f"END,4,{count},{crc:08X}\n".encode()
        return payload, values
    lines = ["CONFIG_OK", "START_OK", f"BEGIN,4,{count},4000000"]
    lines += [f"DATA,4,{i},{v}" for i, v in enumerate(values)]
    lines += [f"END,4,{count},{crc:08X}"]
    return ("\n".join(lines) + "\n").encode(), values


class FragmentDevice:
    def __init__(self, payload, fragment=1, fail_at_end=False):
        self.payload = payload
        self.fragment = fragment
        self.fail_at_end = fail_at_end
        self.offset = 0
        self.eof = threading.Event()
        self.closed = False

    @property
    def in_waiting(self):
        return min(self.fragment, len(self.payload) - self.offset)

    def read(self, size):
        if self.offset == len(self.payload):
            self.eof.set()
            if self.fail_at_end:
                raise OSError("modeled unplug")
            time.sleep(.002)
            return b""
        end = min(len(self.payload), self.offset + self.fragment, self.offset + size)
        chunk = self.payload[self.offset:end]
        self.offset = end
        return chunk

    def write(self, data):
        return len(data)

    def close(self):
        self.closed = True


class StreamTransportTests(unittest.TestCase):
    def test_50000_one_byte_reads_with_paused_gui_and_crc_csv(self):
        payload, values = wire_capture()
        self.check_fragmented_capture(payload, values, 4000)

    def test_binary_50000_one_byte_reads_with_paused_gui_and_crc_csv(self):
        payload, values = wire_capture(binary=True)
        self.check_fragmented_capture(payload, values, 40000)

    def check_fragmented_capture(self, payload, values, rate):
        device = FragmentDevice(payload, fragment=1)
        with tempfile.TemporaryDirectory() as root, patch("trng.transport.SimulatedDevice", return_value=device):
            worker = SerialWorker("SIMULATOR", 115200, root)
            completed, warnings = [], []
            def notify(kind, value):
                if kind == "capture_complete": completed.append(value)
                if kind == "warning": warnings.append(value)
            controller = Controller(lambda data: None, notify)
            controller.configure(Config(adc_rate=rate, samples=50000), start=True)
            framer = WireFramer()
            worker.start()
            # The old app fills 256 one-byte events, then closes after 0.2 s.
            time.sleep(.30)
            peak = worker.incoming.qsize()
            rx_events = 0
            deadline = time.monotonic() + 15
            try:
                while not completed and time.monotonic() < deadline:
                    peak = max(peak, worker.incoming.qsize())
                    budget = time.monotonic() + .010
                    while time.monotonic() < budget:
                        try: kind, data = worker.incoming.get_nowait()
                        except queue.Empty: break
                        if kind == "rx":
                            rx_events += 1
                            self.assertLessEqual(len(data), 4096)
                            for frame_kind, line in framer.feed(data):
                                controller.accept(*parse_line(line) if frame_kind == "line" else (frame_kind, line))
                    if worker.finished.is_set() and worker.incoming.empty(): break
                    time.sleep(.020)  # Approximate the GUI callback cadence.
            finally:
                worker.stop(); worker.join(3)
            self.assertIsNone(worker.error)
            self.assertEqual(worker.incoming.maxsize, 256, "Batch data rather than inflate the event queue")
            self.assertLess(peak, 256)
            self.assertLess(rx_events, 1000)
            self.assertEqual(len(completed), 1, warnings)
            self.assertEqual(completed[0]["samples"], values)
            self.assertEqual(controller.state, "idle")
            self.assertEqual((Path(root)/"rx_raw.bin").read_bytes(), payload)
            store = SessionStore(Path(root)/"sessions", "MODELED_USB", 115200)
            try:
                path = store.capture(completed[0])
                with path.open(encoding="utf-8-sig", newline="") as f:
                    self.assertEqual(sum(1 for _ in csv.DictReader(f)), 50000)
            finally:
                store.close()
            print(f"PASS 50000 fragmented samples: {len(payload)} bytes, {rx_events} GUI RX batches, peak queue {peak}/256")

    def test_unplug_flushes_short_final_fragment(self):
        payload = b"CONFIG_OK\r\nSTART_OK\nBEGIN,1,512,1000000\nDATA,1,0,2048\n"
        device = FragmentDevice(payload, fragment=3, fail_at_end=True)
        with tempfile.TemporaryDirectory() as root, patch("trng.transport.SimulatedDevice", return_value=device):
            worker = SerialWorker("SIMULATOR", 115200, root)
            worker.start(); worker.join(3)
            self.assertTrue(worker.finished.is_set())
            self.assertIn("modeled unplug", worker.error)
            delivered = b"".join(data for kind, data in list(worker.incoming.queue) if kind == "rx")
            self.assertEqual(delivered, payload)
            self.assertEqual((Path(root)/"rx_raw.bin").read_bytes(), payload)

    def test_real_queue_overload_is_explicit_and_raw_bytes_retained(self):
        payload = b"DATA,1,0,2048\n"
        device = FragmentDevice(payload)
        with tempfile.TemporaryDirectory() as root, patch("trng.transport.SimulatedDevice", return_value=device):
            worker = SerialWorker("SIMULATOR", 115200, root)
            worker.incoming = queue.Queue(maxsize=1)
            worker.start(); worker.join(3)
            self.assertTrue(worker.finished.is_set())
            self.assertIn("PC đầy", worker.error)
            self.assertEqual((Path(root)/"rx_raw.bin").read_bytes(), payload)


if __name__ == "__main__":
    unittest.main()
