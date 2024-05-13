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
