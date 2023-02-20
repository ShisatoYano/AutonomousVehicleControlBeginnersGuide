"""
global_xy_visualizer.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt


class GlobalXYVisualizer:
    """
    Visualization class for global 2D X-Y plot
    """
    
    def __init__(self):
        self.objects = [] # register objects here
    
    def add_object(self, aObject):
        """
        Method to register object
        """

        self.objects.append(aObject)
    
    def draw(self):
        # setting
        figure = plt.figure(figsize=(8, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlim(-5, 5)
        axes.set_ylim(-5, 5)
        axes.set_xlabel('X', fontsize=20)
        axes.set_ylabel('Y', fontsize=20)

        # draw registered objects
        for obj in self.objects: obj.draw()

        plt.show()


def main():
    vis = GlobalXYVisualizer()
    vis.draw()


if __name__ == "__main__":
    main()
