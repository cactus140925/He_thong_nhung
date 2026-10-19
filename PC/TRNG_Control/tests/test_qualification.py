"""Bounded automatic hardware qualification and profile identity tests."""
import json
from pathlib import Path
import tempfile
import unittest

from trng.qualification import Qualification, compatible_profile
from trng.storage import SessionStore


class Samples:
    def __init__(self, count): self.count = count
    def __len__(self): return self.count


def result(cfg, **metrics):
    values = dict(fifo_peak=500, copy_max_cycles=2000, pack_max_cycles=1500,
                  capture_ms=round(cfg.samples * 1000 / cfg.adc_rate), core_hz=72000000)
    values.update(metrics)
    return {"samples": Samples(cfg.samples), "rate_millihz": cfg.adc_rate * 1000,
            "adc_rate_hz": cfg.adc_rate, "status": "complete", "crc32": "12345678",
            "pc_elapsed_ms": cfg.samples * 1000 / cfg.adc_rate + 30, "metrics": values}


class QualificationTests(unittest.TestCase):
    def run_suite(self, fail=lambda q, cfg: None, simulation=False):
        q = Qualification("USB:1155:22336:ABC", simulation)
        q.start()
        steps = 0
        while q.active:
            self.assertLess(steps, 40, "The routine must finish, not retry forever")
            if q.recovering:
                q.stop_sent = True
                self.assertIsNone(q.next_config())
                q.acknowledge_stop()
            cfg = q.next_config()
            self.assertIsNotNone(cfg)
            reason = fail(q, cfg)
            if reason:
                q.fail(reason)
            else:
                q.accept_capture(result(cfg))
            steps += 1
        return q

    def test_full_plan_qualifies_max_and_two_long_operating_runs(self):
        q = self.run_suite()
        p = q.report
        self.assertTrue(p["passed"])
        self.assertEqual(p["highest_verified_rate_hz"], 100000)
        self.assertEqual(p["operating_rate_hz"], 80000)
        self.assertEqual(p["max_samples"], 1000000)
        self.assertEqual(len(p["runs"]), 13)
        self.assertEqual([(r["rate_hz"], r["samples"]) for r in p["runs"][-3:]],
                         [(100000, 1000000), (80000, 1000000), (80000, 1000000)])

    def test_failure_in_probe_stops_higher_rates_and_checks_lower_long_run(self):
        q = self.run_suite(lambda q, cfg: "FIFO full" if cfg.adc_rate == 75000 else None)
        self.assertTrue(q.report["passed"])
        self.assertEqual(q.report["highest_verified_rate_hz"], 50000)
        self.assertEqual(q.report["operating_rate_hz"], 40000)
        self.assertFalse(any(r["rate_hz"] == 100000 for r in q.history))

    def test_short_pass_does_not_qualify_a_failing_long_capture(self):
        q = self.run_suite(lambda q, cfg: "long overrun" if q.phase == "limit" and cfg.adc_rate == 100000 else None)
        self.assertEqual(q.report["highest_verified_rate_hz"], 75000)
        self.assertEqual(q.report["operating_rate_hz"], 60000)

    def test_operating_rate_requires_repeat_passes_after_a_downshift(self):
        q = self.run_suite(lambda q, cfg: "host stall" if q.phase == "operating" and cfg.adc_rate == 80000 else None)
        self.assertTrue(q.report["passed"])
        self.assertEqual(q.report["operating_rate_hz"], 64000)
        self.assertEqual(sum(r["pass"] and r["rate_hz"] == 64000 for r in q.history), 2)

    def test_cpu_fifo_time_metrics_and_no_silent_rate_clamping(self):
        for overrides in ({"fifo_peak": 1600}, {"copy_max_cycles": 2_000_000},
                          {"pack_max_cycles": 2_000_000}, {"capture_ms": 3000}):
            q = Qualification("COM26"); q.start(); cfg = q.next_config()
            q.accept_capture(result(cfg, **overrides))
            self.assertFalse(q.active)
            self.assertFalse(q.report["passed"])
        q = Qualification("COM26"); q.start(); cfg = q.next_config()
        data = result(cfg); data["metrics"] = None
        q.accept_capture(data)
        self.assertFalse(q.report["passed"])
        q = Qualification("COM26"); q.start(); cfg = q.next_config()
        data = result(cfg); data.update(rate_millihz=5000000, adc_rate_hz=5000)
        q.accept_capture(data)
        self.assertFalse(q.report["passed"])
        q = Qualification("COM26"); q.start(); cfg = q.next_config()
        data = result(cfg); data["pc_elapsed_ms"] += 1000
        q.accept_capture(data)
        self.assertFalse(q.report["passed"])

    def test_cancel_timeout_and_finite_all_failures(self):
        q = Qualification("COM26"); q.start(); q.next_config(); q.cancel("USB disconnected")
        self.assertIsNone(q.next_config())
        self.assertFalse(q.report["passed"])
        q = self.run_suite(lambda q, cfg: "bad transport")
        self.assertFalse(q.report["passed"])
        self.assertEqual(len(q.history), 1)
        q = self.run_suite(lambda q, cfg: "unstable" if q.phase == "operating" else None)
        self.assertFalse(q.report["passed"])
        self.assertEqual(q.downshifts, 4)

    def test_simulation_cannot_be_reused_as_hardware_and_identity_is_scoped(self):
        p = self.run_suite().report
        self.assertTrue(compatible_profile(p, p["device_key"], "ADC_BIN_2_6"))
        self.assertFalse(compatible_profile(p, "another board", "ADC_BIN_2_6"))
        self.assertFalse(compatible_profile(p, p["device_key"], "ADC_BIN_2_5"))
        sim = self.run_suite(simulation=True).report
        self.assertFalse(sim["cpu_counter_measured"])
        self.assertFalse(compatible_profile(sim, sim["device_key"], "ADC_BIN_2_6"))
        for bad in (None, {}, {**p, "operating_rate_hz": "wrong"}, {**p, "max_samples": 2000000}):
            self.assertFalse(compatible_profile(bad, p["device_key"], "ADC_BIN_2_6"))

    def test_report_persists_only_summaries_not_sample_lists(self):
        p = self.run_suite().report
        with tempfile.TemporaryDirectory() as root:
            store = SessionStore(root, "COM26", 115200)
            try:
                path = store.qualification(p)
                saved = json.loads(path.read_text())
                self.assertEqual(saved["operating_rate_hz"], 80000)
                self.assertLess(path.stat().st_size, 15000)
                self.assertFalse(list(Path(root).rglob("*.part")))
            finally:
                store.close()


if __name__ == "__main__": unittest.main()
