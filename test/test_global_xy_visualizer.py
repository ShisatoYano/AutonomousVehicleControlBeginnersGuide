"""
Unit test of GlobalXYVisualizer

Author: Shisato Yano
"""

import matplotlib
matplotlib.use("Agg")

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/visualization")
import global_xy_visualizer
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters


class FakeObject:
    """
    Object which only counts how many times it was updated
    """

    def __init__(self):
        self.update_count = 0

    def draw(self, axes, elems):
        pass

    def update(self, time_s):
        self.update_count += 1


def test_draw_without_plot_runs_frame_num_frames():
    # smoke tests must run as many frames as the animation does, not a fixed number
    time_params = TimeParameters(span_sec=5)
    vis = GlobalXYVisualizer(MinMax(), MinMax(), time_params)
    obj = FakeObject()
    vis.add_object(obj)
    vis.not_show_plot()

    vis.draw()

    assert obj.update_count == time_params.get_frame_num()


def test_gif_name_shows_animation_without_save_flag(monkeypatch):
    shown = []
    saved = []

    class FakeAnimation:
        def save(self, *args, **kwargs):
            saved.append((args, kwargs))

    monkeypatch.setattr(global_xy_visualizer.anm, "FuncAnimation",
                        lambda *args, **kwargs: FakeAnimation())
    monkeypatch.setattr(global_xy_visualizer.plt, "show",
                        lambda: shown.append(True))
    monkeypatch.setattr(sys, "argv", ["pytest"])

    vis = GlobalXYVisualizer(
        MinMax(), MinMax(), TimeParameters(span_sec=1), gif_name="demo.gif"
    )
    vis.draw()

    assert shown == [True]
    assert saved == []


def test_gif_name_saves_animation_with_save_flag(monkeypatch):
    shown = []
    saved = []

    class FakeAnimation:
        def save(self, *args, **kwargs):
            saved.append((args, kwargs))

    monkeypatch.setattr(global_xy_visualizer.anm, "FuncAnimation",
                        lambda *args, **kwargs: FakeAnimation())
    monkeypatch.setattr(global_xy_visualizer.plt, "show",
                        lambda: shown.append(True))
    monkeypatch.setattr(sys, "argv", ["pytest", "--save-gif"])

    vis = GlobalXYVisualizer(
        MinMax(), MinMax(), TimeParameters(span_sec=1), gif_name="demo.gif"
    )
    vis.draw()

    assert shown == []
    assert saved == [(("demo.gif",), {"writer": "pillow"})]
