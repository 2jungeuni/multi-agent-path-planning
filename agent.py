import random

class Agent(object):
    def __init__(self, idx, x=0, y=0):
        self.id = idx
        self.x = x
        self.y = y
        self.task = None

    def set_task(self, t):
        self.task = t

    def set_random_loc(self, g, grid_size):
        while True:
            rand_x = random.randint(0, grid_size-1)
            rand_y = random.randint(0, grid_size-1)
            if (rand_x, rand_y) not in g.collisions:
                self.x = rand_x
                self.y = rand_y
                break

