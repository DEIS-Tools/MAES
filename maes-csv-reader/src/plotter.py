import matplotlib.pyplot as pyplt
import numpy as np
import plot

class Plotter():
    """
    A class in charge of plotting a set of different plots.
    """
    plots: [plot.Plot]

    def __init__(self, plots):
        """
        Init for Plotter. expects label of x, label of y, a list of list of x, and a list lf list of y.
        Initialises plots automatically.
        """
        self.plots = plots
        self.__plotting()

    def __plotting(self):
        """
        Initialises plots, private method.
        """
        largest_plot_size = 0
        largest_plot = 0

        for i, plot in enumerate(self.plots):
            pyplt.plot(plot.xcords, plot.ycords, label = plot.name)
            print(plot.name)

        pyplt.legend()

        pyplt.xlabel(self.plots[0].x_name)
        pyplt.ylabel(self.plots[0].y_name)

    def show(self):
        """
        Simple wrapper for pyplt.show(), logic can be added if needed.
        """
        pyplt.show()
