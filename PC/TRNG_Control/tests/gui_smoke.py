"""Run with a real Tk display: python tests/gui_smoke.py [screenshot-directory]."""
import csv
from pathlib import Path
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trng.app import App


def pump(app, seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        app.update()
        time.sleep(.01)


def until(app, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app.update()
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("Timed out waiting for GUI state")


def screenshot(app, name):
    if len(sys.argv) < 2:
        return
    from PIL import ImageGrab
    root = Path(sys.argv[1])
    root.mkdir(parents=True, exist_ok=True)
    app.update()
    x, y = app.winfo_rootx(), app.winfo_rooty()
    ImageGrab.grab(bbox=(x, y, x+app.winfo_width(), y+app.winfo_height())).save(root / name)


with tempfile.TemporaryDirectory() as root:
    app = App(data_root=root, load_settings=False)
    callback_errors = []
    app.report_callback_exception = lambda *args: callback_errors.append(args)
    try:
        pump(app, .2)
        app.mode.set("Mô phỏng trên PC")
        app.samples.set("500")
        app.toggle_connection()
        until(app, lambda: app.connected and app.controller.link != "unknown")
        app.rate.set("1000")
        app.samples.set("500")
        assert len(app.tabs.tabs()) == 4
        assert not hasattr(app, "test_uart")
        app.toggle_monitor()
        until(app, lambda: app.controller.monitor)
        app.start_btn.invoke()
        until(app, lambda: app.result_total == 1)
        assert app.controller.state == "idle"
        assert len(app.last_result) == 8
        assert app.adc_total > 0
        assert len(app.table.get_children()) == 1
        assert app.start_btn.winfo_height() >= 30
        assert app.export_btn.winfo_height() >= 30
        assert app.folder_btn.winfo_height() >= 30
        screenshot(app, "app_results.png")
        app.tabs.select(1)
        pump(app, .3)
        screenshot(app, "app_adc.png")
        app.store.export("captures", Path(root)/"export.csv")
        with (Path(root)/"export.csv").open(encoding="utf-8-sig", newline="") as f:
            rows = list(csv.DictReader(f))
        assert rows[0]["samples"] == "500"
        assert rows[0]["source"] == "SIMULATOR"
        assert rows[0]["status"] == "complete"
        # Dedicated RNG tab: simulator runs the SINE9 postprocessor, never
        # claims to be an actual hardware entropy source.
        app.output_mode.set("RNG 9-bit")
        app._output_mode_changed()
        app.samples.set("500")
        app.start_btn.invoke()
        until(app, lambda: app.result_total == 2)
        assert app.controller.state == "idle"
        assert app.store.last_rng_path is not None
        assert app.store.last_rng_path.stat().st_size == 563  # ceil(500*9/8)
        assert app.store.last_rng_path.with_suffix(".json").exists()
        # RNG-progress notification updates its own tab, never the ADC plot.
        old_adc_total, old_adc_points = app.adc_total, list(app.adc_points)
        app._event("capture_block_rng", (0, (11, 24, 511), 1_000_000))
        assert app.adc_total == old_adc_total and list(app.adc_points) == old_adc_points
        assert "Đã nhận 3 từ" in app.rng_stat.get()
        screenshot(app, "app_rng9.png")
        app.output_mode.set("ADC RAW")
        app._output_mode_changed()
        app.samples.set("50000")
        app.start_btn.invoke()
        until(app, lambda: app.controller.state == "running")
        app.stop_btn.invoke()
        until(app, lambda: app.controller.state == "idle")
        assert app.result_total == 2, "STOP must not fabricate a result"
        app.tabs.select(3)
        pump(app, .1)
        screenshot(app, "app_terminal.png")
        app.disconnect()
        until(app, lambda: app.worker is None)
        app.toggle_connection()
        until(app, lambda: app.connected)
        assert app.result_total == 0
        # A disk failure must stop the session and leave the UI responsive.
        def disk_failure(*args):
            raise OSError("simulated disk full")
        app.store.adc = disk_failure
        app.toggle_monitor()
        until(app, lambda: app.worker is None)
        assert app.storage_failed
        app.geometry("1000x720")
        app.tabs.select(0)
        pump(app, .2)
        assert app.start_btn.winfo_height() >= 30
        assert app.export_btn.winfo_height() >= 30
        assert app.folder_btn.winfo_height() >= 30
        screenshot(app, "app_compact.png")
        assert not callback_errors, callback_errors
        print("GUI smoke passed: ADC and RNG9 modes, CRC-verified CSV and .bin, STOP, reconnect, disk-error, 1000x720 layout")
        print("Geometry:", app.winfo_width(), app.winfo_height())
        print("Start button bottom:", app.start_btn.winfo_rooty()+app.start_btn.winfo_height(), "window bottom:", app.winfo_rooty()+app.winfo_height())
    finally:
        if app.worker:
            app.worker.stop()
            app.worker.join(2)
        if app.store:
            app.store.close()
        app.destroy()
