import matplotlib.pyplot as plt
import numpy as np
from collections import namedtuple
from tkinter import *
from time import sleep

PlotObject = namedtuple('PlotObject',
                        ['x',
                         'y',
                         'heading',
                         'radius'])

triangle_shape = [[20, 0],
                  [10, -10],
                  [-10, -10],
                  [-10, 10],
                  [10, 10]]


asset_shape = [[50, 0],
                  [20, -20],
                  [-20, -20],
                  [-20, 20],
                  [20, 20]]

oval_shape = [[-10, -10],
              [10, 10]]

center = [500, 500]

class LivePlot:
    def __init__(self):
        self.window = Tk()
        self.window.geometry("1000x1000")
        self.canvas = Canvas(self.window,
                             width=2*center[0],
                             height=2*center[1])
        self.canvas.pack()
        self.objects = {}

    def create_object(self, sim_obj):
        sim_id = sim_obj.sim_id
        if sim_obj.object_type == "STATIC":
            self.objects[sim_id] = self.canvas.create_oval(oval_shape, fill="YELLOW")
        elif sim_obj.object_type == "USV":
            self.objects[sim_id] = self.canvas.create_polygon(triangle_shape, fill="BLUE")
        elif sim_obj.object_type == "INTRUDER":
            self.objects[sim_id] = self.canvas.create_polygon(triangle_shape, fill="RED")
        elif sim_obj.object_type == "TANKER":
            self.objects[sim_id] = self.canvas.create_polygon(triangle_shape, fill="YELLOW")
        elif som_obj.object_type == "ASSET":
            self.objects[sim_id] = self.canvas.create_polygon(asset_shape)

        return self.objects[sim_id]

    def rotated_triangle_coords(self, angle):
        angle = -angle

        rotate_matrix = [[np.cos(angle),
                          -np.sin(angle)],
                         [np.sin(angle),
                          np.cos(angle)]]
        xy = np.dot(rotate_matrix,
               np.transpose(triangle_shape)).T
        return xy

    def update_object(self, obj, sim_obj):
        sim_id = sim_obj.sim_id
        x = sim_obj.x + center[0]
        y = -sim_obj.y + center[1]
        heading = sim_obj.heading
        obj_type = sim_obj.object_type

        if obj_type in["STATIC", "ASSET"]:
            self.canvas.coords(obj,
                               oval_shape[0][0]+x,
                               oval_shape[0][1]+y,
                               oval_shape[1][0]+x,
                               oval_shape[1][1]+y)
        else:

            xy = self.rotated_triangle_coords(heading)
            xy = xy + np.array([[x, y]])

            self.canvas.coords(obj, *xy.flatten())
            self.objects[sim_id] = obj

    def update_world_state(self, sim_objects):
        for sim_object in sim_objects:
            if sim_object.sim_id in self.objects:
                obj = self.objects[sim_object.sim_id]
            else:
                obj = self.create_object(sim_object)
            self.update_object(obj, sim_object)
            self.canvas.update()
