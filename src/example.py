from holonomic_turtle import HolonomicTurtleRRT
import map as smap
m = smap.SimpleMap(-10, 10, -10, 10, [])
h = HolonomicTurtleRRT((0.0,)*5, 0.5, 0.5, 15, 15, 0.5, m)
h.extend_randomly()
h.extend_randomly()
print map(lambda h: h.data, h.root.children)
