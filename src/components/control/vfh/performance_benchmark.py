"""
performance_benchmark.py

Author: Khushi
"""


class PerformanceBenchmark:
    """
    Live performance benchmarking class for the VFH+ pipeline(Step 7 of
    the VFH roadmap). Each cycle, it reads how long the mapper(histogram
    construction, candidate valley detection and direction selection) and
    the controller(target direction/speed, acceleration and yaw rate)
    each took on their own last update call, and keeps a running mean and
    worst case so the displayed number settles down instead of jittering
    frame to frame.
    """

    def __init__(self, mapper, controller):
        """
        Constructor
        mapper: PolarHistogramMapper instance, read for its own last
                update duration via get_last_update_duration_s()
        controller: VfhController instance, read for its own last update
                    duration via get_last_update_duration_s()
        """

        self.mapper = mapper
        self.controller = controller

        self.frame_count = 0
        self.total_duration_s = 0.0
        self.max_duration_s = 0.0
        self.last_total_duration_s = 0.0

    def update(self, time_s):
        """
        Function to update benchmark statistics from the mapper's and
        controller's own last recorded update durations
        time_s: Simulation interval time[sec]
        """

        mapper_duration_s = self.mapper.get_last_update_duration_s()
        controller_duration_s = self.controller.get_last_update_duration_s()
        self.last_total_duration_s = mapper_duration_s + controller_duration_s

        self.frame_count += 1
        self.total_duration_s += self.last_total_duration_s
        if self.last_total_duration_s > self.max_duration_s:
            self.max_duration_s = self.last_total_duration_s

    def get_last_duration_s(self):
        """
        Function to get the last cycle's combined mapper+controller
        update duration[sec]
        """

        return self.last_total_duration_s

    def get_mean_duration_s(self):
        """
        Function to get the mean per-cycle combined mapper+controller
        update duration[sec] across the whole run so far
        """

        if self.frame_count == 0:
            return 0.0
        return self.total_duration_s / self.frame_count

    def get_max_duration_s(self):
        """
        Function to get the worst-case per-cycle combined mapper+controller
        update duration[sec] seen so far
        """

        return self.max_duration_s

    def draw(self, axes, elems):
        """
        Function to draw a small live readout of the last cycle's update
        time plus the running mean and worst case, in milliseconds
        axes: Axes object of figure
        elems: List of plot objects
        """

        text = ("Step 7 benchmark\n"
               "last: {0:.2f}[ms]\n"
               "mean: {1:.2f}[ms]\n"
               "max: {2:.2f}[ms]").format(self.last_total_duration_s * 1000.0,
                                          self.get_mean_duration_s() * 1000.0,
                                          self.get_max_duration_s() * 1000.0)

        readout = axes.text(0.98, 0.98, text, transform=axes.transAxes,
                            fontsize=9, va="top", ha="right", color="black",
                            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
        elems.append(readout)
