import matplotlib.pyplot as plt
from collections import namedtuple

PlotObject = namedtuple('PlotObject', ['x', 'y'])

class LivePlot:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        plt.ion()
        plt.show(False)

    def update_world_state(self, objects):
        self.fig.clf()
        for obj in objects:
            plt.plot(obj.x, obj.y, 'bx')
            plt.xlim([-10, 10])
            plt.ylim([-10, 10])
            plt.pause(0.1)
        
