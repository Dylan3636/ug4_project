from Tkinter import *
from time import sleep
import math
import numpy as np

window = Tk()
window.geometry("800x800")
c = Canvas(width=800, height=800)
c.pack()

center = 100, 100
func = lambda t : (t[0]+center[0], t[1]+ center[1])
# a square
xy = [(5, 7.5), (0, -7.5), (-5, 7.5)]
xy = list(map(func, xy))
polygon_item = c.create_polygon(xy)


def getangle(event):
    dx = c.canvasx(event.x) - center[0]
    dy = c.canvasy(event.y) - center[1]
    try:
        return complex(dx, dy) / abs(complex(dx, dy))
    except ZeroDivisionError:
        return 0.0 # cannot determine angle

def press(event):
    # calculate angle at start point
    global start
    start = getangle(event)

# calculate current angle relative to initial angle
angle = 0
while True:
    # global start
    angle = math.pi/6.0 # getangle(event) / start
    print(np.rad2deg(angle))
    # angle = angle%(2*math.pi)
    print(angle)
    newxy = []
    for x, y in xy:
        x0, y0 = x-center[0], y-center[1]
        r = math.sqrt(x0**2 + y0**2)
        newxy.append((x0*math.cos(angle)-y0*math.sin(angle)+center[0],
                     x0*math.sin(angle)+y0*math.cos(angle)+ center[1]))
    c.coords(polygon_item, *np.array(newxy).flatten())
    window.update()
    print(newxy)
    xy=newxy
    sleep(0.1)

# c.bind("<Button-1>", press)
# c.bind("<B1-Motion>", motion)
c.mainloop()
