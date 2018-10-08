import matplotlib.pyplot as plt
import numpy as np
from collections import namedtuple
from tkinter import *
from time import sleep

PlotObject = namedtuple('PlotObject',
                        ['x',
                         'y',
                         'heading',
                         'xd',
                         'yd',
                         'offset'])


class LivePlot:
    def __init__(self):
        self.window = Tk()
        self.window.geometry("800x800")
        self.canvas = Canvas(self.window,
                             width=800,
                             height=800)
        self.canvas.pack()
        self.objects = {}

    def create_object(self, sim_obj):
        id = sim_obj.id
        self.objects[id] = self.canvas.create_polygon(triangle_matrix)

    def rotated_triangle(self, angle):
        triangle_shape = [[5, 7.5],
                          [0, -7.5],
                          [-5, 7.5]]

        rotate_matrix = [[np.cos(heading),
                          -np.sin(heading)],
                         [np.sin(heading),
                          np.cos(heading)]]

        xy = np.dot(rotate_matrix,
               np.transpose(triangle_shape)) + np.array([x, y])


    def transform_object(self, sim_obj)
        id = sim_obj.id
        x = sim_obj.x
        y = sim_obj.y
        heading = sim_obj.heading

        obj = rotated_triangle(heading)
        xy = self.canvas(obj)
        xy = np.reshape(xy, [2, None]) + np.array([x, y])
        self.canvas.coord(obj, *xy.flatten())
        self.object[id] = obj
        return

    def update_world_state(self, sim_objects):
        for obj in sim_objects:
            if obj.id not in self.objects:
                ob = self.create_object(obj)
            else:
                ob = self.objects[obj.id]
            xd = obj.speed*np.cos(obj.heading)*0.1
            yd = obj.speed*np.sin(obj.heading)*0.1
            self.canvas.coords(
                ob,
                xd,
                yd)
            sleep(0.1)
            self.canvas.update()
