#!/usr/bin/env python

from Tkinter import *
from PIL import Image, ImageTk
from threading import Lock
import math

import rospy
from std_msgs.msg import String

# we render at half-size everywhere

raw_img = Image.open(sys.argv[1])
width = raw_img.width
height = raw_img.height

root = Tk()
root.title("RRT Visualizer")
canvas = Canvas(root, width=width, height=height)
canvas.pack()

# Turtle Radius
TR = 14.0/2
img = ImageTk.PhotoImage(raw_img)
canvas.create_image(width/2, height/2, image=img)

drawing_lock = Lock()

def draw_turtle(state):
    x, y, r, _, _ = state
    # rotate box coords, and re-center on (x, y)
    #box = [[x + math.cos(r)*cx - math.sin(r)*cy, y + math.sin(r)*cx + math.cos(r)*cy] for cx, cy in [[-TR, TR], [TR, TR], [TR, -TR], [-TR, -TR]]]

    #canvas.create_oval(x, y, x + TURTLE_WIDTH, y + TURTLE_HEIGHT, fill='blue')
    #p = canvas.create_polygon(box, fill='purple')
    #canvas.create_line(x, y, (box[0][0] + box[1][0]) / 2, (box[0][1] + box[1][1]) / 2, width=2, tags='orientation')
    #canvas.tag_raise('orientation', p)

def new_node_cb(line):
    print(line)
    old, control, new = eval(line.data)
    with drawing_lock:
        #draw_turtle(new)
        canvas.create_line(old[0], old[1], new[0], new[1], fill='blue')

rospy.init_node('rrt_visualizer')
rospy.Subscriber('/rrt/new_node', String, new_node_cb)
root.mainloop()
