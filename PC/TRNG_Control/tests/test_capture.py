import csv
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

from trng.controller import Controller
from trng.protocol import Config, LineFramer, parse_line
from trng.storage import SessionStore


def frames(count=512, session=3):
    values = [(i * 29) & 4095 for i in range(count)]
    crc = zlib.crc32(b"".join(struct.pack("<H", value) for value in values))
    return ([f"BEGIN,{session},{count},1000000"] +
            [f"DATA,{session},{i},{value}" for i, value in enumerate(values)] +
            [f"END,{session},{count},{crc:08X}"]), values, crc


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.sent, self.events = [], []
        self.c = Controller(self.sent.append, lambda k, v: self.events.append((k, v)), clock=lambda: self.now)

    def start(self, count=512):
        self.c.configure(Config(samples=count), start=True)
        self.assertNotEqual(self.sent[-1], b"START\n")
        self.c.accept("config_ok", None)
        self.c.accept("start_ok", None)

    def results(self):
        return [v for k, v in self.events if k == "capture_complete"]

    def test_chunks_to_crc_and_exact_csv(self):
        self.start()
        lines, values, crc = frames()
        framer = LineFramer()
        payload = ("\r\n".join(lines) + "\r\n").encode()
        for offset in range(0, len(payload), 7):
            for kind, line in framer.feed(payload[offset:offset + 7]):
                self.c.accept(*parse_line(line))
        result, = self.results()
        self.assertEqual(result["samples"], values)
        self.assertEqual(result["crc32"], f"{crc:08X}")
        self.assertEqual(self.c.state, "idle")
        with tempfile.TemporaryDirectory() as root:
            store = SessionStore(root, "COM26", 115200)
            try:
                path = store.capture(result)
                with path.open(encoding="utf-8-sig", newline="") as f:
                    rows = list(csv.DictReader(f))
                self.assertEqual(len(rows), 512)
                self.assertEqual(rows[511]["index"], "511")
                self.assertEqual(rows[1]["time_s"], "0.001000000")
                exported = Path(root) / "export.csv"
                store.export("adc", exported)
                self.assertEqual(path.read_bytes(), exported.read_bytes())
                self.assertFalse(list(store.path.glob("*.part")))
            finally:
                store.close()

    def test_corrupt_missing_duplicate_session_and_end(self):
        original, _, _ = frames()
        variants = []
        a = original.copy(); del a[40]; variants.append(a)
        a = original.copy(); a.insert(40, a[40]); variants.append(a)
        a = original.copy(); a[40], a[41] = a[41], a[40]; variants.append(a)
        a = original.copy(); a[40] = a[40].replace("DATA,3,", "DATA,4,"); variants.append(a)
        a = original.copy(); a[-1] = "END,3,512,FFFFFFFF"; variants.append(a)
        a = original.copy(); a[-1] = "END,3,510,00000000"; variants.append(a)
        a = original.copy(); a[40] = "DATA,3,39,4096"; variants.append(a)
        for lines in variants:
            with self.subTest(line=lines[40]):
                self.setUp(); self.start()
                for line in lines:
                    self.c.accept(*parse_line(line))
                self.assertFalse(self.results())
                self.assertEqual(self.c.state, "uncertain")

    def test_missing_end_times_out_without_result(self):
        self.start()
        lines, _, _ = frames()
        for line in lines[:-1]:
            self.c.accept(*parse_line(line))
        self.assertFalse(self.results())
        self.now = 20
        self.c.tick()
        self.assertEqual(self.c.state, "uncertain")

    def test_stop_drops_queued_frames_until_ack(self):
        self.start()
        lines, _, _ = frames()
        self.c.accept(*parse_line(lines[0]))
        self.c.stop()
        for line in lines[1:]:
            self.c.accept(*parse_line(line))
        self.c.accept("error", "BUSY")
        self.assertFalse(self.results())
        self.assertEqual(self.c.state, "stopping")
        self.c.accept("stop_ok", None)
        self.assertEqual(self.c.state, "idle")
        self.start(count=256)

    def test_firmware_errors_and_old_stop_response(self):
        for text in ("ERR=SAMPLE_EVEN_2_TO_1000000", "ERROR:SAMPLE_EVEN_2_TO_512", "ERROR,3,13"):
            self.setUp(); self.start()
            self.c.accept(*parse_line(text))
            self.assertEqual(self.c.state, "uncertain")
            self.assertFalse(self.results())
        self.c.stop()
        self.c.accept(*parse_line("ERROR,3,9"))
        self.assertEqual(self.c.state, "idle")

    def test_limits_match_firmware(self):
        for cfg in (Config(samples=513), Config(samples=514), Config(samples=65534), Config(samples=1_000_002), Config(adc_rate=100_001),
                    Config(adc_rate=100_001, samples=50000), Config(size=256)):
            with self.assertRaises(ValueError):
                cfg.validate()
        self.assertEqual(Config(samples=50000).validate().samples, 50000)
        self.assertEqual(Config(adc_rate=40000, samples=512).validate().adc_rate, 40000)
        for rate in (4001, 8000, 10000, 20000, 30000, 40000, 40001, 50000, 60000, 75000, 80000, 100000):
            self.assertEqual(Config(adc_rate=rate, samples=50000).validate().adc_rate, rate)
        self.assertEqual(parse_line("BEGINB,1,500000,100000000")[0], "capture_begin_binary")
        self.assertEqual(parse_line("BEGINB,1,500000,100000001")[0], "parse_error")

    def test_preview_not_part_of_capture(self):
        self.c.accept(*parse_line("ADC=1000,2048"))
        self.assertFalse(self.results())
        self.assertIn(("adc", (1000, 2048)), self.events)

    def test_runtime_metrics_bind_to_capture_and_validate_shape(self):
        self.start()
        lines, _, _ = frames()
        for line in lines[:-1]: self.c.accept(*parse_line(line))
        metrics = "MTR,3,500,720,1440,512,72000000"
        self.c.accept(*parse_line(metrics))
        self.c.accept(*parse_line(lines[-1]))
        self.assertEqual(self.results()[0]["metrics"]["fifo_peak"], 500)
        for bad in ("MTR,3,2049,1,1,512,72000000", "MTR,3,500,1,1,512,0", "MTR,3,500"):
            self.assertEqual(parse_line(bad)[0], "parse_error")
        self.setUp(); self.start(); self.c.accept(*parse_line(lines[0]))
        self.c.accept(*parse_line(metrics)); self.c.accept(*parse_line(metrics))
        self.assertEqual(self.c.state, "uncertain")
        self.assertFalse(self.results())


if __name__ == "__main__":
    unittest.main()
