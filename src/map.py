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

# A map representing a bounded, continuous 2D world with discrete obstacles
class SimpleMap(object):
    def __init__(self, xmin, xmax, ymin, ymax, obstacles):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.obstacles = obstacles

    def collides_any(self, point):
        return False

    def random_free_point(self):
        x, y = random.uniform(self.xmin, self.xmax), random.uniform(self.ymin, self.ymax)
        while self.collides_any((x, y)):
            x, y = random.uniform(self.xmin, self.xmax), random.uniform(self.ymin, self.ymax)
        return (x, y)
