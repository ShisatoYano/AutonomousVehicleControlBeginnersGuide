"""
state_history.py

Author: Shisato Yano
"""


class StateHistory:
    """
    Vehicle state(x, y)'s history data and logic class
    """

    def __init__(self, x_history, y_history):
        """
        Constructor
        x_history: List of x in vehicle's state
        y_history: List of y in vehicle's state
        """
        
        self.x_history = x_history
        self.y_history = y_history
    
    def update(self, x, y):
        """
        Function to update x-y's history
        """

        updated_x_history = self.x_history
        updated_y_history = self.y_history

        updated_x_history.append(x)
        updated_y_history.append(y)

        return StateHistory(updated_x_history, updated_y_history)
    
    def draw(self, axes, color, elems):
        """
        Function draw x-y history's trajectory
        """
        
        elems += axes.plot(self.x_history, self.y_history, linewidth=0, marker='.', color=color)
