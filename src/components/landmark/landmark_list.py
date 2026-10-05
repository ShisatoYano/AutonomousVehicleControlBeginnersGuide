"""
landmark_list.py

Collection of landmarks (mirrors obstacle.ObstacleList).
"""


class LandmarkList:
    """
    Holds Landmark objects; supports drawing and iteration for sensors.
    """

    def __init__(self):
        self.list = []

    def add_landmark(self, landmark):
        """Append a Landmark instance."""
        self.list.append(landmark)

    def get_list(self):
        return self.list

    def __iter__(self):
        return iter(self.list)

    def __len__(self):
        return len(self.list)

    def update(self, _time_s):
        """
        Landmarks are static; this lets LandmarkList be added to the visualizer.
        """
        pass

    def draw(self, axes, elems, label_first_only=True):
        """
        Draw all landmarks. If label_first_only, only the first gets a legend label.
        """
        for i, lm in enumerate(self.list):
            lm.draw(
                axes,
                elems,
                label="True landmarks" if (label_first_only and i == 0) else None,
            )

    def as_xy_tuples(self):
        """List of (x, y) for plotting or legacy paths."""
        return [(lm.get_x_m(), lm.get_y_m()) for lm in self.list]
