import numpy as np
import matplotlib.pyplot as plt
from helper_tools import *

angles = np.arange(0, 2*np.pi, np.pi/6)

for angle in angles:
    ax = plt.figure().add_subplot(111)
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    center = (2*np.cos(angle), 2*np.sin(angle))
    c = plt.Circle(center, 1)
    ax.add_artist(c)
    pts = edge_points_of_circle((0, 0), center, 1)
    print(center)
    print(pts)
    ax.plot([0, pts[0][0]], [0, pts[0][1]], 'g-.')
    ax.plot([0, pts[1][0]], [0, pts[1][1]], 'r-.')
    plt.show()
