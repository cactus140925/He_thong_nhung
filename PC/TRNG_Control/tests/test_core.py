import csv
import os
from pathlib import Path
import queue
import sys
import tempfile
import time
import unittest

from trng.controller import Controller
from trng.protocol import Config, LineFramer, ResultAssembler, parse_line
from trng.storage import SessionStore
from trng.transport import SerialWorker, SimulatedDevice


class ProtocolTests(unittest.TestCase):
    def test_document_config_exact(self):
        self.assertEqual(Config().wire(), b"CMD=CONFIG\nADC_RATE=1000\nSAMPLE=512\nSIZE=32\n")

    def test_bad_config_bounds(self):
        for cfg in [Config(0), Config(samples=0), Config(size=33), Config(size=8192), Config(adc_rate=10_000_001)]:
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                cfg.wire()

    def test_fragmented_crlf(self):
        f = LineFramer()
        self.assertEqual(f.feed(b"DA"), [])
        self.assertEqual(f.feed(b"TA=AB\r"), [])
        self.assertEqual(f.feed(b"\nTIME=2\nSAMPLE=4\n"), [("line", "DATA=AB"), ("line", "TIME=2"), ("line", "SAMPLE=4")])

    def test_overflow_discards_whole_line_and_resyncs(self):
        f = LineFramer(limit=8)
        self.assertEqual(f.feed(b"X" * 100)[0][0], "parse_error")
        self.assertLessEqual(len(f.buffer), 8)
        self.assertEqual(f.feed(b"START_OK\nSTOP_OK\n"), [("line", "STOP_OK")])

    def test_invalid_bytes(self):
        self.assertEqual(LineFramer().feed(b"\xff\n")[0][0], "parse_error")

    def test_reject_nonfinite_negative_corrupted(self):
        for line in ["TIME=nan", "TIME=inf", "TIME=-1", "SAMPLE=-1", "ADC=5,6,7", "DATA=xyz", "ADC=4294967296", "DATA=" + "F"*1025]:
            with self.subTest(line=line[:50]):
                self.assertEqual(parse_line(line)[0], "parse_error")

    def test_adc_formats_and_hex(self):
        self.assertEqual(parse_line("ADC=12,2048"), ("adc", (12, 2048)))
        self.assertEqual(parse_line("ADC=2048"), ("adc", (None, 2048)))
        self.assertEqual(parse_line("DATA=0xaabb"), ("data", "AABB"))

    def test_result_reverse_metadata_order(self):
        a = ResultAssembler()
        self.assertIsNone(a.accept("data", "AB"))
        self.assertIsNone(a.accept("sample", 100))
        self.assertEqual(a.accept("time", 1.5), {"random_hex": "AB", "samples": 100, "time_ms": 1.5})
        self.assertIsNone(a.pending)

    def test_duplicate_data_not_overwritten(self):
        a = ResultAssembler()
        a.accept("data", "AA")
        with self.assertRaises(ValueError):
            a.accept("data", "BB")


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.sent, self.events = [], []
        self.c = Controller(self.sent.append, lambda *event: self.events.append(event), lambda: self.now)

    def start_run(self):
        self.c.configure(Config(samples=10, size=32), start=True)
        self.assertEqual(len(self.sent), 1)
        self.c.accept("config_ok", None)
        self.assertEqual(self.sent[-1], b"START\n")
        self.c.accept("start_ok", None)
        self.assertEqual(self.c.state, "running")

    def test_start_requires_config_ack(self):
        self.c.configure(Config(), start=True)
        self.assertNotIn(b"START\n", self.sent)
        self.now = 3.1
        self.c.tick()
        self.assertEqual(self.c.state, "uncertain")
        self.c.accept("config_ok", None)
        self.assertNotIn(b"START\n", self.sent)

    def test_complete_result_uses_device_time(self):
        self.start_run()
        self.now = 2.0
        for line in ["DATA=1234", "TIME=10.5", "SAMPLE=10"]:
            self.c.accept(*parse_line(line))
        row = [value for event, value in self.events if event == "result"][0]
        self.assertEqual(row["random_hex"], "00001234")
        self.assertEqual(row["pc_elapsed_ms"], 2000.0)
        self.assertEqual(row["time_ms"], 10.5)
        self.assertEqual(row["status"], "complete")
        self.assertEqual(self.c.state, "idle")

    def test_stop_before_late_config_ack(self):
        self.c.configure(Config(), start=True)
        self.c.stop()
        self.c.accept("config_ok", None)
        self.assertNotIn(b"START\n", self.sent)
        self.c.accept("stop_ok", None)
        self.assertEqual(self.c.state, "idle")

    def test_stop_race_completed_result_keeps_waiting_for_ack(self):
        self.start_run()
        self.c.stop()
        for line in ["DATA=0", "TIME=10", "SAMPLE=10"]:
            self.c.accept(*parse_line(line))
        self.assertEqual(self.c.state, "stopping")
        self.c.accept("stop_ok", None)
        self.assertEqual(self.c.state, "idle")

    def test_missing_fields_not_exported(self):
        self.start_run()
        self.c.accept("data", "AB")
        self.now = 3.1
        self.c.tick()
        self.assertEqual(self.c.state, "uncertain")
        self.assertFalse(self.c.run_active)
        self.c.accept("time", 1)
        self.c.accept("sample", 10)
        self.assertFalse(any(k == "result" for k, _ in self.events))

    def test_duplicate_result_invalidates_run(self):
        self.start_run()
        for line in ["DATA=AA", "DATA=BB", "TIME=1", "SAMPLE=10"]:
            self.c.accept(*parse_line(line))
        self.assertFalse(any(k == "result" for k, _ in self.events))
        self.assertEqual(self.c.state, "uncertain")

    def test_mismatch_flag(self):
        self.start_run()
        for line in ["DATA=123456789", "TIME=10", "SAMPLE=9"]:
            self.c.accept(*parse_line(line))
        row = [v for k, v in self.events if k == "result"][0]
        self.assertEqual(row["status"], "mismatch")

    def test_begin_rejects_wrong_sample_rate_even_with_valid_frame_crc(self):
        self.start_run()
        self.c.accept("capture_begin_binary", (1, 10, 2000000))  # requested 1000 Hz
        self.assertEqual(self.c.state, "uncertain")
        self.assertFalse(self.c.run_active)
        self.assertFalse(any(k == "capture_complete" for k, _ in self.events))

    def test_begin_accepts_timer_divider_rounding(self):
        self.start_run()
        self.c.accept("capture_begin_binary", (1, 10, 1000500))
        self.assertEqual(self.c.state, "running")
        self.assertEqual(self.c.capture.rate_millihz, 1000500)

    def test_monitor_extension_failure_is_recoverable(self):
        self.c.toggle_monitor()
        self.c.accept("error", "UNKNOWN_COMMAND")
        self.assertEqual(self.c.state, "idle")
        self.assertIsNone(self.c.monitor_pending)

    def test_device_error_requires_stop(self):
        self.start_run()
        self.c.accept("error", "ADC_FAILURE")
        self.assertEqual(self.c.state, "uncertain")
        self.assertFalse(self.c.run_active)
        self.c.stop()
        self.c.accept("stop_ok", None)
        self.assertEqual(self.c.state, "idle")


class StoreTests(unittest.TestCase):
    def test_sessions_do_not_overwrite_and_csv_escapes(self):
        with tempfile.TemporaryDirectory() as root:
            a = SessionStore(root, "SIMULATOR", 115200)
            b = SessionStore(root, "COM7", 115200)
            self.assertNotEqual(a.path, b.path)
            a.event("RX", "=HYPERLINK(1)")
            a.adc(1, None, 2000)
            a.result({"run_id": 1, "random_hex": "01", "time_ms": 10, "samples": 50, "status": "complete"})
            dst = Path(root) / "copy.csv"
            a.export("events", dst)
            with dst.open(encoding="utf-8-sig", newline="") as f:
                row = list(csv.DictReader(f))[0]
            self.assertEqual(row["detail"], "'=HYPERLINK(1)")
            a.close()
            b.close()
            a.export("adc", Path(root)/"adc.csv")


class TransportTests(unittest.TestCase):
    def wait_event(self, worker, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        seen = []
        while time.monotonic() < deadline:
            try:
                e = worker.incoming.get(timeout=.05)
            except queue.Empty:
                if worker.finished.is_set():
                    self.fail(worker.error or "Worker stopped")
                continue
            seen.append(e)
            if predicate(e):
                return e
        self.fail(f"Timeout; seen={seen}")

    def test_simulated_device_end_to_end(self):
        with tempfile.TemporaryDirectory() as root:
            w = SerialWorker("SIMULATOR", 115200, root)
            w.start()
            try:
                self.wait_event(w, lambda e: e[0] == "connected")
                w.send(Config(adc_rate=1000, samples=50, size=64).wire())
                self.wait_event(w, lambda e: e[0] == "rx" and b"CONFIG_OK" in e[1])
                w.send(b"START\n")
                e = self.wait_event(w, lambda e: e[0] == "rx" and b"END," in e[1])
                self.assertIn(b"\xA5\x5A\xC3\x3C", e[1])
                self.assertIn(b"END,1,50,", e[1])
            finally:
                w.stop()
                w.join(2)
            self.assertFalse(w.is_alive())
            self.assertIsNone(w.error)
            self.assertIn(b"START\n", (Path(root)/"tx_raw.bin").read_bytes())

    @unittest.skipIf(sys.platform == "win32", "PTY integration uses POSIX")
    def test_real_pyserial_pty_fragment_and_disconnect(self):
        import pty
        import select
        master, slave = pty.openpty()
        master_closed = False
        try:
            with tempfile.TemporaryDirectory() as root:
                w = SerialWorker(os.ttyname(slave), 115200, root)
                w.start()
                try:
                    self.wait_event(w, lambda e: e[0] == "connected")
                    w.send(Config(samples=2, size=32).wire())
                    select.select([master], [], [], 2)
                    self.assertEqual(os.read(master, 1024), Config(samples=2, size=32).wire())
                    os.write(master, b"CONFIG_")
                    self.wait_event(w, lambda e: e[0] == "rx")
                    os.write(master, b"OK\r\nDATA=DEADBEEF\nTIME=2\nSAMPLE=2\n")
                    self.wait_event(w, lambda e: e[0] == "rx" and b"SAMPLE" in e[1])
                    os.close(master)
                    master_closed = True
                    w.join(2)
                    self.assertTrue(w.finished.is_set())
                    self.assertIsNotNone(w.error)
                    self.assertIn(b"CONFIG_OK\r\n", (Path(root)/"rx_raw.bin").read_bytes())
                finally:
                    w.stop()
                    w.join(2)
        finally:
            if not master_closed:
                os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
