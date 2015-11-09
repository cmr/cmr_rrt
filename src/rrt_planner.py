#!/usr/bin/env python2

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import math

from PIL import Image

import rospy
from std_msgs.msg import String

from holonomic_turtle import HolonomicTurtleRRT
from rrt import REACHED
from map import SimpleMap


init = eval(sys.argv[1])
goal = eval(sys.argv[2])

print("Ok, navigating from %s to %s!" % (init, goal))

img = Image.open(sys.argv[3])

m = SimpleMap(0.0, float(img.width), 0.0, float(img.height), img)

print("Loading collision data from %s..." % sys.argv[3])

#for y in range(img.height):
#    for x in range(img.width):
#        if img.getpixel((x, y)) == (255, 255, 255, 255):
#            m.add_obstacle((x, y), (x, y))


print("Initializing RRT...")

rrt = HolonomicTurtleRRT(init, 0.5, math.pi/32, 3.0, math.pi/3, 1.0, m)

rospy.init_node('rrt_planner')
p = rospy.Publisher('/rrt/new_node', String, queue_size=10)

print("Here we go!")

i = 0
rate = rospy.Rate(500)
while not rospy.is_shutdown():
    state, new_node = rrt.extend_randomly()
    if new_node is not None:
        p.publish("(%s, %s, %s)" % (new_node.parent.data, (new_node.edge_data[0], new_node.edge_data[1]), new_node.data))
    i += 1
    if i % 50 == 0:
        state, new_node = rrt.extend(goal)
        if new_node is not None:
            p.publish("(%s, %s, %s)" % (new_node.parent.data, (new_node.edge_data[0], new_node.edge_data[1]), new_node.data))
        if state is REACHED:
            print "We made it!"
            break
    rate.sleep()
