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

import random

from rtree import index

# A map representing a bounded, continuous 2D world with discrete rectangular
# obstacles.

class SimpleMap(object):
    # obstacles should be a list of tuples representing the lower left and
    # upper right bounds of the obstacle.
    def __init__(self, xmin, xmax, ymin, ymax, obstacles):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        p = index.Property()
        p.dimension = 2
        self.spidx = index.Index(properties=p)
        self.max = 0
        self.invspidx = {}
        self.obstimg = obstacles

    def add_obstacle(self, mi, ma):
        self.spidx.insert(self.max, mi + ma)
        self.invspidx[self.max] = (mi, ma)
        self.max += 1

    def collides_img(self, pt):
        if not self.inbounds((pt[0]-3, pt[1]-3), (pt[0]+3, pt[1]+3)):
            return True
        for x in range(-3, 3):
            for y in range(-3, 3):
                try:
                    if all(map(lambda c: c == 0, self.obstimg.getpixel((int(pt[0])+x, int(pt[1])+y))[:3])):
                        return True
                except IndexError:
                    print "Wtf IndexError? It's inbounds! %s" % str((pt[0], pt[1]))
                    return True
        return False

    def inbounds(self, mi, ma, dbg=True):
        if mi[0] < self.xmin or mi[0] >= self.xmax or ma[0] < self.xmin or ma[0] >= self.xmax:
            return False
        if mi[1] < self.ymin or mi[1] >= self.ymax or ma[1] < self.ymin or ma[1] >= self.ymax:
            return False
        return True

    def collides_any(self, mi, ma, dbg=True):
        self.inbounds(mi, ma, dbg)
        l = list(self.spidx.intersection(mi, ma))
        if l and dbg:
            print "(%s, %s) intersected with these things: %s" % (mi, ma, l)
        return bool(l)

    def random_free_point(self):
        x, y = random.uniform(self.xmin, self.xmax), random.uniform(self.ymin, self.ymax)
        while self.collides_img((x, y)):
            x, y = random.uniform(self.xmin, self.xmax), random.uniform(self.ymin, self.ymax)
        return (x, y)
