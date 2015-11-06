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

# simple tree with parent-pointers and data associated with each node and
# edge.

class Node(object):
    def __init__(self, data=None, parent=None):
        self.data = data
        self.children = []
        self.parent = None
        self.edge_data = None

    def add_child(self, child, edge_data=None):
        if child not in self.children:
            child.parent = self
            child.edge_data = edge_data
            self.children.append(child)

    def detach(self):
        try:
            idx = self.parent.children.remove(self)
        except ValueError as e:
            print "Help!! My parent doesn't think I am its child :("
            raise e

        self.parent = None
        self.edge_data = None

    def find_root(self):
        cur = self.parent
        while cur.parent is not None:
            cur = cur.parent
        return cur
