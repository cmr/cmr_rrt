import random
import math
import operator

from scipy import integrate, optimize

import rrt

# A RRT specialized to a "holonomic turtle", a robot in 2D whose pose is (x,
# y, theta) representing the position and orientation on a map, with kinematic
# parameters (posvel, angvel) representing a positive or negative velocity in
# the direction of the current orientation and a speed at which the robot is
# rotating. Our state space is simply our 5D configuration space.
#
# We have a max positional and angular velocity, as well as a max angular and
# positional acceleration. One timestep is given, which is how often we change
# the control input to the robot.

# (NOTE: This doesn't apply now that I'm using scipy - leaving here in case I
# switch back)
# Two timesteps are given. One is a timestep for
# the kinematic simulation integrator (sparts), and one is how long we apply an action
# to determine the new state from an old state (tdt). tdt represents how often
# we change the acceleration, whereas sdt determines how accurate the
# kinematic simulation is. If sdt is small enough, this can compensate for a
# relatively large tdt, however it will cause the number of nodes needed in
# the tree to explode before there is a good probability of reaching the goal
# state. The actual timestep for the integration is implicit - it is equal to
# tdt/sparts, where sparts represents the number of intervals of tdt to use.
# At each tdt interval, we can chose an acceleration.

# TODO: use ndarrays more often?

def step(x_orig, interval, alpha, beta):
    def r(t):
        return x_orig[2] + x_orig[4]*t + (alpha * t**2)/2

    def x(t):
        return x_orig[0] + math.cos(x_orig[2]) * x_orig[3] * t + (math.cos(r(t)) * beta * t**2)/2

    def y(t):
        return x_orig[1] + math.sin(x_orig[2]) * x_orig[3] * t + (math.sin(r(t)) * beta * t**2)/2

    def a(t):
        return x_orig[4] + alpha * t

    def p(t):
        return x_orig[3] + beta * t

    # toss out the error terms. probably not important anyway, amirite ;)
    q = lambda f: integrate.quad(f, 0, interval)[0]

    return (q(x), q(y), q(r), q(p), q(a))

class HolonomicTurtleRRT(rrt.RRT):
    def __init__(self, initial_configuration, max_dp, max_dr, max_vel, max_avel, dt, map):
        super(HolonomicTurtleRRT, self).__init__()
        self.map = map
        self.root.data = initial_configuration
        self.max_vel = max_vel
        self.max_avel = max_avel
        self.max_dp = max_dp
        self.max_dr = max_dr
        self.dt = dt
        self.root.edge_data = (0, 0)

    def random_position_velocity(self):
        return random.uniform(-self.max_vel, self.max_vel)

    def random_angular_velocity(self):
        return random.uniform(-self.max_avel, self.max_avel)

    def random_state(self):
        x, y = self.map.random_free_point()
        r = random.uniform(0, 2*math.pi)
        pv = self.random_position_velocity()
        av = self.random_angular_velocity()
        return (x, y, r, pv, av)

    # use a k-d tree
    def nearest_neighbor(self, x):
        return self.root

    # Given a configuration (x0, y0, r0, p0, a0), to try and reach
    # (x1, y1, r1, p1, a1), we will ... do something. What we want to do is
    # minimize  x_dst - x_new, given a particular choice for our accelerations
    # alpha and beta.
    #
    # We will be under a constant acceleration the entire time, and our motion
    # model is:
    #
    # r(t) = r_0 + a_0 * t + (alpha * t**2)/2
    # ... (see step above)
    #
    # I then run scipy's optimizer to minimize the sum of the square of x_dst
    # - step(...). I'm not sure of this is optimal, but it's at least one way
    # to collapse the vector into a scalar for scipy.
    def new_state(self, src_node, x_dst):
        x_src = src_node.data
        def objective(x):
            alpha, beta = x
            return math.fsum(
                    map(lambda x: x**2,
                        map(lambda (a, b): a - b, zip(x_dst, step(x_src, self.dt, alpha, beta)))
                        )
                    )

        res = optimize.minimize(objective, (0, 0),
            bounds=((-self.max_dr, self.max_dr), (-self.max_dp, self.max_dp)))

        x_new = step(x_src, self.dt, res.x[0], res.x[1])
        return x_new, res.x

    # close enough if each component of abs(a-b) less than some epsilon
    # (chosen to be 1e-12 as the approximate error of the integration with
    # even somewhat long (multiple-dozen seconds) timesteps is < 1e-12)
    def close_enough(self, a, b):
        map(abs, map(lambda (a, b): a - b, zip(a, b))) < (1e-12,)*len(a)
