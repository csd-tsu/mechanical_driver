from mechanics_module import trajectory
from numpy import sqrt

class Entity:
    def __init__(self, m, x0, vx0, y0, vy0, shape):
        self.x = []
        self.vx = []
        self.y = []
        self.vy = []
        self.m = m
        self.shape = shape
        self.initial_vector = [x0, vx0, y0, vy0]

    def trajectory(self, t, gravity):
        self.x, self.vx, self.y, self.vy = trajectory(self.initial_vector, self.x, self.vx, self.y, self.vy, t, gravity)

    def slice(self, t):
        self.x = self.x[:t]
        self.vx = self.vx[:t]
        self.y = self.y[:t]
        self.vy = self.vy[:t]


class Circle(Entity):
    def __init__(self, m, x0, vx0, y0, vy0, shape, r, c_recovery):
        Entity.__init__(self, m, x0, vx0, y0, vy0, shape)
        self.r = r
        self.c_recovery = c_recovery

class Rectangle(Entity):
    def __init__(self, m, x0, vx0, y0, vy0, shape, width, height):
        Entity.__init__(self, m, x0, vx0, y0, vy0, shape)
        self.width = width
        self.height = height
        self.diagonal = sqrt(self.width**2 + self.height**2)
