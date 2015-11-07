An implementation of the RRT algorithm as presented in [1], and an application
of it to a model of a robot in the 2D plane with a positional and angular
velocity.

Status:

- [x] Core RRT algorithm
- [x] Turtlebot motion model
- [x] Nearest-neighbor query for turtlebot
- [ ] Convex collision for `SimpleMap`.
- [ ] Visualizer of the RRT as it's built
- [ ] Multi-tree planner
- [ ] Integrate planner into ROS control node
- [ ] RRT parameter tweak GUI
- [ ] Instructions for running.
- [ ] Investigate efficacy of `scipy.optimize.minimize` in this usecase.

[1] - http://msl.cs.uiuc.edu/~lavalle/papers/LavKuf01b.pdf
