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

class Plot:
    """
    Class to hold plots (list of x-y coordinates)
    """
    name: str
    cords: [(float, float)]
    xcords: [float]
    ycords: [float]
    x_name: str
    y_name: str

    def __init__(self, name, xcords, ycords, x_name, y_name):
        self.name = name
        self.cords = []
        self.xcords = xcords
        self.ycords = ycords
        self.x_name = x_name
        self.y_name = y_name
        self.make_plot()

    def make_plot(self):
        for i in range(len(self.xcords)):
            self.cords.append((self.xcords[i], self.ycords[i]))

if __name__ == '__main__':
    plot = Plot([1,2,3], [53,70,90])
    print(plot.cords)
