"""Binary protocol integrity, recovery and GUI block-delivery regressions."""
import csv
from collections import deque
import random
import struct
import tempfile
import unittest
import zlib
from types import SimpleNamespace

from trng.app import App
from trng.controller import Controller
from trng.protocol import (BINARY_FRAME_BYTES, BINARY_SAMPLES, Config, WireFramer,
                           encode_binary_block, parse_line)
from trng.storage import SessionStore


def capture(count=512, session=13, rate=100000):
    # Includes LF, CR, NUL and each magic byte in the sample low bytes.
    pattern = (0, 10, 13, 165, 90, 195, 60, 4095)
    values = [pattern[i % len(pattern)] for i in range(count)]
    packets = [encode_binary_block(session, i, values[i:i + BINARY_SAMPLES])
               for i in range(0, count, BINARY_SAMPLES)]
    crc = zlib.crc32(struct.pack(f"<{count}H", *values))
    begin = f"BEGINB,{session},{count},{rate * 1000}\n".encode()
    end = f"END,{session},{count},{crc:08X}\n".encode()
    return begin, packets, end, values


class BinaryTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.events = []
        self.c = Controller(lambda data: None, lambda k, v: self.events.append((k, v)), clock=lambda: self.now)
        self.framer = WireFramer()

    def start(self, count=512):
        self.c.configure(Config(adc_rate=100000, samples=count), start=True)
        self.c.accept("config_ok", None)
        self.c.accept("start_ok", None)

    def feed(self, payload, chunk=7):
        for offset in range(0, len(payload), chunk):
            for kind, value in self.framer.feed(payload[offset:offset + chunk]):
                self.c.accept(*(parse_line(value) if kind == "line" else (kind, value)))

    def results(self):
        return [v for k, v in self.events if k == "capture_complete"]

    def test_split_every_byte_full_blocks_tail_and_csv(self):
        for chunk in (1, 2, 7, 62, 64, 4096):
            with self.subTest(chunk=chunk):
                self.setUp(); self.start()
                begin, packets, end, values = capture()
                self.assertTrue(all(len(p) == BINARY_FRAME_BYTES for p in packets))
                self.feed(begin + b"".join(packets) + end, chunk)
                result, = self.results()
                self.assertEqual(result["samples"], values)
                self.assertEqual(self.c.state, "idle")
                self.assertFalse([k for k, _ in self.events if k == "capture_sample"])
                self.assertEqual(sum(k == "capture_block" for k, _ in self.events), len(packets))
        with tempfile.TemporaryDirectory() as root:
            store = SessionStore(root, "COM26", 115200)
            try:
                path = store.capture(result)
                with path.open(encoding="utf-8-sig", newline="") as file:
                    rows = list(csv.DictReader(file))
                self.assertEqual(len(rows), len(values))
                self.assertEqual([int(r["adc_raw"]) for r in rows], values)
            finally:
                store.close()

    def test_frame_crc_covers_header_samples_padding_and_stop_recovery(self):
        begin, packets, end, _ = capture()
        for byte in (0, 1, 4, 5, 6, 8, 12, 16, 40, 58, 61):
            for chunk in (1, 137):
                with self.subTest(byte=byte, chunk=chunk):
                    self.setUp(); self.start()
                    bad = bytearray(packets[0]); bad[byte] ^= 1
                    self.feed(begin + bad + b"".join(packets[1:]) + end, chunk)
                    self.assertEqual(self.c.state, "uncertain")
                    self.assertFalse(self.results())
                    self.c.stop(); self.feed(b"STOP_OK\n", chunk)
                    self.assertEqual(self.c.state, "idle")
        # Tail-frame padding is covered even though it is not an ADC sample.
        self.setUp(); self.start()
        bad = bytearray(packets[-1]); bad[50] ^= 1
        self.feed(begin + b"".join(packets[:-1]) + bad + end)
        self.assertFalse(self.results())

    def test_dropped_inserted_and_truncated_bytes_never_complete(self):
        begin, packets, end, _ = capture()
        variants = (packets[0][:20] + packets[0][21:], packets[0][:20] + b"X" + packets[0][20:])
        for first in variants:
            self.setUp(); self.start()
            self.feed(begin + first + b"".join(packets[1:]) + end)
            self.assertFalse(self.results())
            self.c.stop(); self.feed(b"STOP_OK\n", 1)
            self.assertEqual(self.c.state, "idle")
        self.setUp(); self.start(); self.feed(begin + packets[0][:30])
        self.now = 11; self.c.tick()
        self.assertEqual(self.c.state, "uncertain")
        self.assertFalse(self.results())

    def test_missing_duplicate_wrong_session_index_and_dataset_crc(self):
        begin, packets, end, values = capture()
        variants = [packets[1:], [packets[0], *packets], [packets[1], packets[0], *packets[2:]],
                    [encode_binary_block(14, 0, values[:21]), *packets[1:]],
                    [encode_binary_block(13, 1, values[:21]), *packets[1:]]]
        for blocks in variants:
            self.setUp(); self.start(); self.feed(begin + b"".join(blocks) + end)
            self.assertFalse(self.results())
            self.assertEqual(self.c.state, "uncertain")
        self.setUp(); self.start()
        self.feed(begin + b"".join(packets) + b"END,13,512,00000000\n")
        self.assertFalse(self.results())

    def test_stop_discards_inflight_then_next_session_passes(self):
        self.start()
        begin, packets, end, values = capture()
        self.feed(begin + packets[0])
        self.c.stop()
        self.feed(b"".join(packets[1:]) + end + b"STOP_OK\n")
        self.assertEqual(self.c.state, "idle")
        self.assertFalse(self.results())
        self.start(); self.feed(begin + b"".join(packets) + end)
        self.assertEqual(self.results()[0]["samples"], values)

    def test_text_binary_mixing_rejected_and_legacy_supported(self):
        begin, packets, end, _ = capture()
        for header, data in ((b"BEGIN,13,512,100000000\n", packets[0]),
                             (begin, b"DATA,13,0,0\n"), (b"", packets[0])):
            self.setUp(); self.start(); self.feed(header + data + end)
            self.assertFalse(self.results())
            self.assertEqual(self.c.state, "uncertain")
        self.setUp(); self.start(count=2)
        crc = zlib.crc32(struct.pack("<2H", 10, 4095))
        self.feed(f"BEGIN,1,2,100000000\nDATA,1,0,10\nDATA,1,1,4095\nEND,1,2,{crc:08X}\n".encode())
        self.assertEqual(self.results()[0]["samples"], [10, 4095])

    def test_bounded_junk_and_random_chunk_large_capture(self):
        self.framer = WireFramer(limit=128)
        self.assertTrue(self.framer.feed(b"X" * 100000))
        self.assertLessEqual(len(self.framer.buffer), 128)
        self.setUp(); self.start(count=50000)
        begin, packets, end, values = capture(50000)
        payload = begin + b"".join(packets) + end
        rng = random.Random(24)
        offset = 0
        while offset < len(payload):
            size = rng.randint(1, 4096)
            self.feed(payload[offset:offset + size], size)
            offset += size
        self.assertEqual(self.results()[0]["samples"], values)
        text_size = sum(len(f"DATA,13,{i},{v}\n") for i, v in enumerate(values))
        self.assertLess(len(payload), text_size / 5)
        print(f"PASS binary byte count: {len(payload)} vs text DATA {text_size}, {text_size / len(payload):.2f}x smaller")

    def test_gui_block_callback_keeps_all_counts_and_bounded_plot(self):
        app = SimpleNamespace(adc_total=0, adc_points=deque(maxlen=2500), capture_received=0,
                              sample_stat_dirty=False)
        for first in range(0, 500000, 21):
            values = tuple((i * 29) & 4095 for i in range(first, min(first + 21, 500000)))
            App._event(app, "capture_block", (first, values, 40000000))
        self.assertEqual(app.adc_total, 500000)
        self.assertEqual(app.capture_received, 500000)
        self.assertTrue(app.sample_stat_dirty)
        self.assertEqual(len(app.adc_points), 2500)
        self.assertEqual(app.adc_points[-1], (499999, (499999 * 29) & 4095))


if __name__ == "__main__":
    unittest.main()
