import numpy as np
from collections import namedtuple
from tkinter import *
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

oval_shape = [[-5, -5],
              [5, 5]]

center = [1000, 500]


def rgb_to_hexa(*args):
    """
    Convert RGB(A) color to hexadecimal.
    ref: tkcolorpicker
    """
    print(args)
    if len(args) == 3:
        return ("#{:02x}{:02x}{:02x}".format(*args)).upper()
    elif len(args) == 4:
        return ("#{:02x}{:02x}{:02x}{:02x}".format(*tuple(args))).upper()
    else:
        raise ValueError("Wrong number of arguments.")


def threat_prob_to_hex_color(p):
    r = round(255*p)
    g = round(255*(1-p))
    hex_col = rgb_to_hexa(r, g, 0)
    print("HEX: ", hex_col)
    return hex_col


class LivePlot:
    def __init__(self):
        self.window = Tk()
        self.window.geometry("2000x1000")
        self.canvas = Canvas(self.window,
                             width=2000,
                             height=1000, bg='white')
        # self.scale = Scale(self.window, orient=HORIZONTAL, length=1000, from_=0, to=1000)
        # self.scale.pack()
        self.canvas.pack(fill=BOTH, expand=True)
        self.canvas.bind('<Configure>', self.create_grid)
        self.objects = {}
        self.markers = {}
        self.lines = {}
        self.thread_lock = Lock()
        self.queue = Queue()
        self.threat_queue = Queue()

    def threat_color_callback(self, msg):
        self.threat_queue.put(msg)

    def update_threat_color(self):
        while not self.threat_queue.empty():
            with self.thread_lock:
                obj = self.threat_queue.get()
                print(obj)
                self.canvas.itemconfig(self.objects[obj.intruder_id],
                                       fill=threat_prob_to_hex_color(obj.threat_probability))

    def create_grid(self, event=None):
        w = self.canvas.winfo_width() # Get current width of canvas
        h = self.canvas.winfo_height() # Get current height of canvas
        self.canvas.delete('grid_line') # Will only remove the grid_line

        # Creates all vertical lines at intevals of 100
        for i in range(0, w, 100):
            self.canvas.create_line(i, 0, i, h, tag='grid_line')

        # Creates all horizontal lines at intevals of 100
        for i in range(0, h, 100):
            self.canvas.create_line(0, i, w, i, tag='grid_line')

    def create_object(self, sim_obj):
        sim_id = sim_obj.sim_id
        if sim_obj.object_type == "STATIC":
            self.objects[sim_id] = self.canvas.create_oval(oval_shape, fill="YELLOW")
        elif sim_obj.object_type == "USV":
            self.objects[sim_id] = self.canvas.create_polygon(triangle_shape, fill="BLUE")
        elif sim_obj.object_type == "INTRUDER":
            self.objects[sim_id] = self.canvas.create_polygon(triangle_shape, fill=threat_prob_to_hex_color(0))
        elif sim_obj.object_type == "TANKER":
            self.objects[sim_id] = self.canvas.create_polygon(triangle_shape, fill="YELLOW")
        elif sim_obj.object_type == "ASSET":
            self.objects[sim_id] = self.canvas.create_polygon(asset_shape)

        return self.objects[sim_id]

    @staticmethod
    def rotated_triangle_coords(angle):
        angle = -angle

        rotate_matrix = [[np.cos(angle),
                          -np.sin(angle)],
                         [np.sin(angle),
                          np.cos(angle)]]
        xy = np.dot(rotate_matrix,
                    np.transpose(triangle_shape)).T
        return xy

    @staticmethod
    def offset_object_position(x, y):
        return x + center[0], -y + center[1]

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

            xy = self.rotated_triangle_coords(heading) / (2 if obj_type != "TANKER" else 1)
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
                del self.objects[sim_id]
