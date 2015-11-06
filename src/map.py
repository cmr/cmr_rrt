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
