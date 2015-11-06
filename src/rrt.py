import tree

# TODO: this ought to be an ABC.

REACHED = object()
ADVANCED = object()
TRAPPED = object()

# An implementation of the rapidly-exploring random tree (RRT).
#
# See the original paper (http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)
# for the motivation and details. This implementation is very generic. To
# instantiate this algorithm, create a subclass and implement the required
# methods. The contracts they must obey are documented on each method. This
# implementation focuses on clarity and ease of implementation, and is not
# particularly fast.
class RRT(object):
    def __init__(self):
        self.root = tree.Node()

    # This method should return a random point in X_free, using
    # whatever distribution and soforth you please.
    def random_state(self):
        raise NotImplementedError()

    # This method should somehow determine the nearest vertex to x in the
    # current RRT, returning the Node object.
    def nearest_neighbor(self, x):
        raise NotImplementedError()

    # This method takes a node in the tree and a destination state and should
    # return a pair, (x_new, u_new) where x_new is the new state and u_new is
    # the input applied to x to reach x_new. There are no particular
    # properties required, but it is my understanding that if u_new minimizes
    # the distance to x_near, the search will converge faster.
    # x_new *must* be in X_free, and u_new *must* be a valid input.
    # If there is no valid (x_new, u_new) (or you can't find one), return
    # None.
    def new_state(self, x_src, x_dst):
        raise NotImplementedError()

    # This method determines whether a is "close enough" to b to be considered
    # the same state. This is used to determine whether or not the RRT
    # extension in fact reached the destination state.
    def close_enough(self, a, b):
        raise NotImplementedError()

    def extend_randomly(self):
        self.extend(self.random_state())
    # This method should not be overridden. This method takes a destination
    # state x_dst and tries to extend the tree towards x_dst.
    #
    # This will return TRAPPED if the nearest_neighbor query failed, ADVANCED
    # if a node was added to the tree but we could not quite reach the
    # destination, or REACHED if we got close_enough to the destination
    def extend(self, x_dst):
        x_near = self.nearest_neighbor(x_dst)
        ns = self.new_state(x_near, x_dst)
        if ns is None:
            return TRAPPED
        x_new, u_new = ns
        x_near.add_child(tree.Node(x_new), edge_data = u_new)
        if self.close_enough(x_near.data, x_dst):
            return REACHED
        return ADVANCED
