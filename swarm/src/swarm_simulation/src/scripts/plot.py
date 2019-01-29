import matplotlib.pyplot as plt
import numpy as np
from collections import namedtuple
from tkinter import *
from time import sleep
from threading import Lock
from queue import Queue
PlotObject = namedtuple('PlotObject',
                        ['x',
                         'y',
                         'heading',
                         'radius'])

triangle_shape = [[20, 0],
                  [10, -10],
                  [-20, -10],
                  [-20, 10],
                  [10, 10]]


asset_shape = [[30, 0],
                  [20, -20],
                  [-30, -20],
                  [-30, 20],
                  [20, 20]]

oval_shape = [[-10, -10],
              [10, 10]]

center = [1000, 500]

class LivePlot:
    def __init__(self):
        self.window = Tk()
        self.window.geometry("2000x1000")
        self.canvas = Canvas(self.window,
                             width=2000,#20*center[0],
                             height=1000)#20*center[1])
        self.canvas.pack()
        self.objects = {}
        self.markers = {}
        self.lines = {}
        self.thread_lock = Lock()
        self.queue = Queue()

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

    def offset_object_position(self, x, y):
        return (x +center[0], -y + center[1])

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

            xy = self.rotated_triangle_coords(heading) /( 2 if obj_type != "TANKER" else 1)
            xy = xy + np.array([[x, y]])

            self.canvas.coords(obj, *xy.flatten())
            self.objects[sim_id] = obj

    def update_world_state(self, sim_objects):
        # Update vessels
        for sim_id, sim_object in sim_objects.items():
            if sim_object.sim_id in self.objects:
                obj = self.objects[sim_object.sim_id]
            else:
                obj = self.create_object(sim_object)
            self.update_object(obj, sim_object)
        self.thread_lock.acquire()
        
        # Update Lines
        for sim_id, line in self.lines.items():
            if sim_id not in self.objects:
                start_obj = sim_objects[line.start_id]
                end_obj = sim_objects[line.end_id]
                start_xy = self.offset_object_position(start_obj.x, start_obj.y) 
                end_xy = self.offset_object_position(end_obj.x, end_obj.y) 
                self.objects[sim_id] = self.canvas.create_line(*start_xy, *end_xy)
            else:
                start_obj = sim_objects[line.start_id]
                end_obj = sim_objects[line.end_id]
                start_xy = self.offset_object_position(start_obj.x, start_obj.y) 
                end_xy = self.offset_object_position(end_obj.x, end_obj.y) 
                self.canvas.coords(self.objects[sim_id],
                                   *start_xy,
                                   *end_xy)


        # Update Markers
        for sim_id, marker in self.markers.items():
            if marker.sim_id not in self.objects:
                self.objects[sim_id] = self.canvas.create_oval(oval_shape, fill=marker.colour)
            obj = self.objects[sim_id]
            x = marker.x + center[0]
            y = -marker.y + center[1]
            self.canvas.coords(obj,
                               oval_shape[0][0]+x,
                               oval_shape[0][1]+y,
                               oval_shape[1][0]+x,
                               oval_shape[1][1]+y)
        self.thread_lock.release()
        self.canvas.update()
    
    def update_marker(self, marker):
        with self.thread_lock:
            self.markers[marker.sim_id] = marker
    def update_lines(self, line_updates):
        set_1 = set(self.lines.keys())
        set_2 = set(line_updates.keys())
        with self.thread_lock:
            for sim_id in set_1.difference(set_2):
                self.queue.put(sim_id)
            self.lines = line_updates
    def cull_objects(self):
        while not self.queue.empty():
            sim_id = self.queue.get()
            with self.thread_lock:
                self.canvas.delete(self.objects[sim_id])
                self.objects[sim_id]=None