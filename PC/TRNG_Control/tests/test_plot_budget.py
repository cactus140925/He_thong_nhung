"""Exercise the real plot preparation without requiring a desktop display."""
from collections import deque
from types import SimpleNamespace
import unittest

from trng.app import App


class Canvas:
    def __init__(self): self.traces = []
    def winfo_width(self): return 1900
    def winfo_height(self): return 500
    def delete(self, *args): pass
    def create_text(self, *args, **kwargs): pass
    def create_line(self, *args, **kwargs):
        if kwargs.get("width") == 1.6: self.traces.append(args)


class PlotBudgetTests(unittest.TestCase):
    def test_large_window_keeps_spike_with_bounded_render_work(self):
        values = [(i, 2048) for i in range(2500)]
        values[1234] = (1234, 4095)
        canvas = Canvas()
        app = SimpleNamespace(canvas=canvas, tabs=SimpleNamespace(index=lambda x: 1),
                              adc_points=deque(values), adc_bits=SimpleNamespace(get=lambda: "12"),
                              adc_stat=SimpleNamespace(set=lambda text: None), adc_total=50000)
        App.draw_plot(app)
        self.assertEqual(len(canvas.traces), 1)
        trace = canvas.traces[0]
        self.assertLessEqual(len(trace) // 2, 1200)
        self.assertIn(20.0, trace[1::2], "Preserve a narrow full-scale spike")


if __name__ == "__main__": unittest.main()
