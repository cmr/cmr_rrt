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

from holonomic_turtle import HolonomicTurtleRRT
import map as smap
m = smap.SimpleMap(-10, 10, -10, 10, [])
h = HolonomicTurtleRRT((0.0,)*5, 0.5, 0.5, 15, 15, 0.5, m)
h.extend_randomly()
h.extend_randomly()
print map(lambda h: h.data, h.root.children)
