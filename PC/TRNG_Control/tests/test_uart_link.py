"""UART and CDC share capture controls; port choice and INFO set their limits."""
from pathlib import Path
from types import MethodType, SimpleNamespace
import json
import tempfile
import unittest

from trng.app import App, MODE_CDC, MODE_UART, MODE_SIM, MODE_OUTPUT_RNG, UART_RATES, CDC_RATES
from trng.controller import Controller
from trng.protocol import Config, parse_line
from trng.qualification import Qualification


class Value:
    def __init__(self, value=""): self.value, self.options = value, {}
    def set(self, value): self.value = value
    def get(self): return self.value
    def configure(self, **kwargs): self.options.update(kwargs)


def app_harness():
    events, sent = [], []
    h = SimpleNamespace(connected=False, disconnecting=False, worker=None, controller=None,
                        qualifier=None, saved_profile=None, device_info="", current_device_key="PORT:COM_TTL",
                        store=SimpleNamespace(event=lambda *a: events.append(a)), append_log=lambda *a: None,
                        mode=Value(MODE_CDC), port=Value("COM_TTL"), baud=Value("115200"),
                        rate=Value("50000"), samples=Value("100000"), bits=Value("32"))
    for name in ("qualification_status", "notice", "status", "connection_hint", "mode_badge", "mode_combo",
                 "port_combo", "baud_combo", "dtr_check", "scan_btn", "folder_btn", "rate_entry",
                 "sample_entry", "bits_combo", "config_btn", "start_btn", "monitor_btn", "qualify_btn",
                 "stop_btn", "manual_btn", "connect_btn", "estimate_label"):
        setattr(h, name, Value())
    for name in ("_event", "_can_qualify", "_display_link", "_rate_limit", "_refresh_transport_ui",
                 "_on_mode_changed", "_controls", "_estimate", "current_config"):
        setattr(h, name, MethodType(getattr(App, name), h))
    h.controller = Controller(sent.append, h._event)
    return h, events, sent


class UARTLinkTests(unittest.TestCase):
    def setUp(self):
        self.sent, self.events = [], []
        self.now = 0.0
        self.c = Controller(self.sent.append, lambda *event: self.events.append(event), lambda: self.now)

    def info(self, link="UART"):
        rate = 3000 if link == "UART" else 100000
        self.c.accept(*parse_line(f"INFO=ADC_BIN_2_7;HZ={rate};MAX=1000000;FIFO=2048;LINK={link}"))

    def test_uart_limit_is_enforced_before_any_config_bytes_are_sent(self):
        self.info()
        with self.assertRaises(ValueError): self.c.configure(Config(3001, 10000), start=True)
        self.assertFalse(self.sent)
        self.c.configure(Config(3000, 10000), start=True)
        self.assertIn(b"ADC_RATE=3000", self.sent[-1])
        self.c.accept("config_ok", None)
        self.assertEqual(self.sent[-1], b"START\n")

    def test_usb_limit_and_legacy_info_remain_supported(self):
        self.info("USB")
        self.c.configure(Config(100000, 1000000))
        self.assertIn(b"ADC_RATE=100000", self.sent[-1])
        legacy = Controller(self.sent.append, lambda *args: None)
        legacy.accept("info", "ADC_BIN_2_6;HZ=100000;MAX=1000000;FIFO=2048")
        self.assertEqual((legacy.link, legacy.max_adc_rate), ("USB", 100000))

    def test_mode_switch_changes_defaults_presets_and_estimate_before_connect(self):
        h, _, _ = app_harness()
        h.mode.set(MODE_UART); h._on_mode_changed()
        self.assertEqual((h.rate.get(), h.samples.get(), h.baud.get()), ("1000", "10000", "115200"))
        self.assertEqual(h.rate_entry.options["values"], UART_RATES)
        self.assertIn("3,000", h.estimate_label.options["text"])
        self.assertIn("PA9→RX", h.connection_hint.get())
        self.assertEqual(h.port.get(), "")
        h.rate.set("3001")
        with self.assertRaises(ValueError): h.current_config()
        h.mode.set(MODE_CDC); h._on_mode_changed()
        self.assertEqual((h.rate.get(), h.samples.get()), ("50000", "100000"))
        self.assertEqual(h.rate_entry.options["values"], CDC_RATES)

    def test_port_switch_preserves_valid_rng_rate(self):
        h, _, _ = app_harness()
        h.output_mode = Value(MODE_OUTPUT_RNG)
        h.mode.set(MODE_UART); h._on_mode_changed()
        self.assertEqual(h.rate.get(), "1000")
        h.mode.set(MODE_CDC); h._on_mode_changed()
        self.assertEqual(h.rate.get(), "1000")
        self.assertEqual(h.current_config().mode, "rng9")

    def test_controls_wait_for_info_then_use_uart_on_shared_dashboard(self):
        h, _, _ = app_harness()
        h.connected = True; h.worker = SimpleNamespace()
        h._controls()
        self.assertEqual(h.start_btn.options["state"], "disabled")
        h.controller.accept("info", "ADC_BIN_2_7;HZ=3000;MAX=1000000;FIFO=2048;LINK=UART")
        self.assertEqual(h.mode.get(), MODE_UART)
        self.assertEqual((h.rate.get(), h.samples.get()), ("1000", "10000"))
        self.assertEqual(h.start_btn.options["state"], "normal")
        self.assertEqual(h.monitor_btn.options["state"], "normal")
        self.assertEqual(h.qualify_btn.options["state"], "disabled")
        self.assertIn("UART (USB–TTL)", h.mode_badge.options["text"])
        self.assertIn("COM_TTL", h.mode_badge.options["text"])
        self.assertFalse(hasattr(App, "test_uart"))
        self.assertFalse(hasattr(Controller, "test_uart"))

    def test_info_identifies_cdc_even_when_user_selected_uart(self):
        h, _, _ = app_harness()
        h.connected = True; h.worker = SimpleNamespace(); h.mode.set(MODE_UART)
        h.controller.accept("info", "ADC_BIN_2_7;HZ=100000;MAX=1000000;FIFO=2048;LINK=USB")
        self.assertEqual(h.mode.get(), MODE_CDC)
        self.assertIn("là USB CDC", h.notice.get())
        self.assertEqual((h.rate.get(), h.samples.get()), ("50000", "100000"))
        self.assertEqual(h.qualify_btn.options["state"], "normal")

    def test_repeated_info_preserves_user_config_and_simulation_identity(self):
        h, _, _ = app_harness(); h.connected = True; h.worker = SimpleNamespace()
        line = "ADC_BIN_2_7;HZ=3000;MAX=1000000;FIFO=2048;LINK=UART"
        h.controller.accept("info", line)
        h.rate.set("2000"); h.samples.set("50000")
        h.controller.accept("info", line)
        self.assertEqual((h.rate.get(), h.samples.get()), ("2000", "50000"))
        h2, _, _ = app_harness(); h2.mode.set(MODE_SIM); h2.connected = True
        h2.controller.accept("info", "ADC_BIN_2_7;HZ=100000;MAX=1000000;FIFO=2048;LINK=USB")
        self.assertEqual(h2.mode.get(), MODE_SIM)
        self.assertIn("MÔ PHỎNG", h2.mode_badge.options["text"])

    def test_repeated_info_does_not_reset_manual_rate_with_saved_profile(self):
        h, _, _ = app_harness()
        h.connected = True; h.worker = SimpleNamespace()
        h.current_device_key = "PORT:COM_TTL"
        h.saved_profile = {"schema": 1, "passed": True, "simulation": False,
                           "device_key": h.current_device_key, "firmware": "ADC_BIN_2_7",
                           "operating_rate_hz": 80000, "highest_verified_rate_hz": 100000,
                           "max_samples": 1000000}
        line = "ADC_BIN_2_7;HZ=100000;MAX=1000000;FIFO=2048;LINK=USB"
        h.controller.accept("info", line)
        self.assertEqual(h.rate.get(), "80000")
        h.rate.set("32000"); h.samples.set("50000")
        h.controller.accept("info", line)
        self.assertEqual((h.rate.get(), h.samples.get()), ("32000", "50000"))

    def test_invalid_uart_rate_does_not_replace_last_config(self):
        self.info()
        self.c.configure(Config(1000, 10000))
        self.c.accept("config_ok", None)
        self.assertEqual(self.c.config.adc_rate, 1000)
        with self.assertRaises(ValueError):
            self.c.configure(Config(3001, 10000))
        self.assertEqual(self.c.config.adc_rate, 1000)

    def test_saved_uart_choice_restores_connection_mode(self):
        h, _, _ = app_harness(); h.mode.set(MODE_UART); h._on_mode_changed()
        with tempfile.TemporaryDirectory() as root:
            h.persist_settings = True; h.folder = Value(root); h.config_path = Path(root)/"settings.json"
            App.save_settings(h)
            data = json.loads(h.config_path.read_text())
            self.assertEqual((data["connection_mode"], data["adc_rate"], data["samples"]), (MODE_UART, 1000, 10000))

    def test_qualification_report_retains_the_actual_firmware_tag(self):
        q = Qualification("USB:board", firmware="ADC_BIN_2_7")
        q.start(); q.cancel()
        self.assertEqual(q.report["firmware"], "ADC_BIN_2_7")


if __name__ == "__main__": unittest.main()
