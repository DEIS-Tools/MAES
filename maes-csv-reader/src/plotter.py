# Copyright 2024 MAES
# 
# This file is part of MAES
# 
# MAES is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.
# 
# MAES is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with MAES. If not, see http:#www.gnu.org/licenses/.
# 
# Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
# 
# Original repository: https:#github.com/Molitany/MAES

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
