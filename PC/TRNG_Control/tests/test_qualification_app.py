"""Real App qualification callbacks without a desktop; protocol/CSV tested separately."""
from collections import deque
from pathlib import Path
import tempfile
import json
from types import MethodType, SimpleNamespace
import unittest

from trng.app import App
from trng.controller import Controller
from trng.protocol import Config
from trng.storage import SessionStore
from test_qualification import result


class Widget:
    def __init__(self, value=""): self.value = value
    def set(self, value): self.value = value
    def get(self): return self.value
    def configure(self, **kwargs): pass
    def select(self, *args): pass
    def insert(self, *args, **kwargs): return "row"
    def get_children(self): return []


class QualificationAppTests(unittest.TestCase):
    def harness(self, root):
        session = SessionStore(root, "COM26", 115200)
        saved_csv, writes = [], []
        store = SimpleNamespace(path=session.path, event=session.event, qualification=session.qualification,
                                capture=lambda data: saved_csv.append(len(data["samples"])) or Path(root)/"capture.csv")
        h = SimpleNamespace(connected=True, disconnecting=False, controller=None, qualifier=None,
                            qual_reported=False, device_info="ADC_BIN_2_6;HZ=100000;MAX=1000000;FIFO=2048",
                            current_device_key="USB:1:2:ABC", worker=SimpleNamespace(port="COM26"),
                            saved_profile=None, store=store, progress={}, result_total=0, result_map={},
                            adc_points=deque(maxlen=2500), adc_total=0, table=Widget(), tabs=Widget(),
                            _controls=lambda: None, append_log=lambda *args: None, set_random=lambda *args: None,
                            save_settings=lambda: writes.append(True), _warn=lambda value: None)
        for name in ("rate", "samples", "bits", "qualification_status", "notice", "status", "sample_stat",
                     "time_stat", "result_stat", "monitor_btn"):
            setattr(h, name, Widget())
        callbacks, sent = [], []
        h.after = lambda delay, callback: callbacks.append(callback)
        for name in ("_event", "start_qualification", "_advance_qualification", "_finish_qualification", "stop_device", "_can_qualify"):
            setattr(h, name, MethodType(getattr(App, name), h))
        h.controller = Controller(sent.append, h._event)
        return h, session, callbacks, sent, saved_csv, writes

    def drive(self, h, callbacks, fail_rate=None):
        count = 0
        while h.qualifier.active or callbacks:
            count += 1
            self.assertLess(count, 100)
            while callbacks:
                callbacks.pop(0)()
            if not h.qualifier.active: break
            if h.controller.state == "stopping":
                h.controller.accept("stop_ok", None)
                continue
            self.assertEqual(h.controller.state, "configuring")
            cfg = h.qualifier.current
            h.controller.accept("config_ok", None)
            h.controller.accept("start_ok", None)
            if cfg.adc_rate == fail_rate and h.qualifier.phase == "probe":
                h.controller.accept("capture_error", (h.controller.run_id, 13))
                continue
            data = result(cfg)
            data.update(run_id=h.controller.run_id, requested_samples=cfg.samples,
                        device_session=h.controller.run_id, wire_format="binary")
            h.controller.run_active = False
            h.controller.set_state("idle")
            h._event("capture_complete", data)

    def test_one_click_advances_and_saves_one_final_csv_and_profile(self):
        with tempfile.TemporaryDirectory() as root:
            h, session, callbacks, sent, csv_counts, writes = self.harness(root)
            try:
                h.start_qualification()
                self.drive(h, callbacks)
                self.assertTrue(h.qualifier.report["passed"])
                self.assertEqual(h.rate.get(), "80000")
                self.assertEqual(h.samples.get(), "100000")
                self.assertEqual(csv_counts, [1000000])
                self.assertEqual(len(writes), 1)
                self.assertEqual(sum(data.startswith(b"CMD=CONFIG") for data in sent), 13)
                self.assertTrue((session.path/"qualification.json").exists())
            finally: session.close()

    def test_fifo_failure_runs_stop_ack_then_lower_rate(self):
        with tempfile.TemporaryDirectory() as root:
            h, session, callbacks, sent, _, _ = self.harness(root)
            try:
                h.start_qualification(); self.drive(h, callbacks, fail_rate=100000)
                self.assertIn(b"STOP\n", sent)
                self.assertEqual(h.qualifier.report["highest_verified_rate_hz"], 75000)
                self.assertEqual(h.rate.get(), "60000")
            finally: session.close()

    def test_stop_cancels_without_restarting_and_old_firmware_blocked(self):
        with tempfile.TemporaryDirectory() as root:
            h, session, callbacks, sent, csv_counts, writes = self.harness(root)
            try:
                h.device_info = "ADC_BIN_2_5"
                h.start_qualification(); self.assertIsNone(h.qualifier)
                h.device_info = "ADC_BIN_2_6"
                h.start_qualification(); h.stop_device()
                h.controller.accept("stop_ok", None)
                while callbacks: callbacks.pop(0)()
                self.assertFalse(h.qualifier.active)
                self.assertFalse(h.qualifier.report["passed"])
                self.assertEqual(sum(data.startswith(b"CMD=CONFIG") for data in sent), 1)
                self.assertFalse(csv_counts or writes)
            finally: session.close()

    def test_simulated_rate_and_unfinished_probe_are_not_saved_as_operating_settings(self):
        with tempfile.TemporaryDirectory() as root:
            h = SimpleNamespace(persist_settings=True, qualifier=None, saved_profile=None,
                                mode=Widget("Mô phỏng trên PC"), baud=Widget("115200"), port=Widget("COM26"),
                                folder=Widget(root), config_path=Path(root)/"settings.json",
                                current_config=lambda: Config(100000, 1000000))
            App.save_settings(h)
            data = json.loads(h.config_path.read_text())
            self.assertEqual(data["adc_rate"], 50000)
            self.assertEqual(data["samples"], 100000)
            h.qualifier = SimpleNamespace(active=True)
            h.config_path.unlink()
            App.save_settings(h)
            self.assertFalse(h.config_path.exists())

    def test_unsuccessful_recheck_invalidates_the_old_profile_for_this_board(self):
        with tempfile.TemporaryDirectory() as root:
            h, session, callbacks, sent, _, writes = self.harness(root)
            try:
                h.saved_profile = {"device_key": h.current_device_key, "passed": True}
                h.start_qualification(); h.stop_device()
                self.assertIsNone(h.saved_profile)
                self.assertEqual(h.rate.get(), "50000")
                self.assertTrue(writes)
            finally: session.close()


if __name__ == "__main__": unittest.main()
