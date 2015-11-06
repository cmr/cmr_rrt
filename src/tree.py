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
