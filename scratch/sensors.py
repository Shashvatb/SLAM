import pygame
import math
import numpy as np


class laserSensor:
    def __init__(self, range, map, uncertainty):
        self.range = range
        self.speed = 4 # rounds per second
        self.sigma = np.array([uncertainty[0], uncertainty[1]])
        self.pos = (0, 0)
        self.map = map
        self.W, self.H = pygame.display.get_surface().get_size()
        self.sensor_obstacles = []

    def get_distance(self, obstacle_position):
        distance = np.linalg.norm(np.array(obstacle_position) - np.array(self.pos))
        return distance

    def add_uncertainty(self, distance, angle, sigma):
        mean = np.array([distance, angle])
        cov = np.diag(sigma**2)
        distance, angle = np.random.multivariate_normal(mean, cov)
        distance = max(distance, 0)
        return [distance, angle]
    
    def sense_obstacles(self, samples=100):
        data = []
        x1, y1 = self.pos
        for angle in np.linspace(0, 2*math.pi, 120, False):
            x2, y2 = (x1 + self.range*math.cos(angle), y1 - self.range*math.sin(angle))
            for i in range(0, samples):
                u = i/100
                x = int(x2*u + x1*(1-u))
                y = int(y2*u + y1*(1-u))
                if 0 < x < self.W and 0 < y < self.H:
                    color = self.map.get_at((x,y))
                    if (color[0], color[1], color[2]) == (0,0,0):
                        distance = self.get_distance((x,y))
                        output = self.add_uncertainty(distance, angle, self.sigma)
                        output.append(self.pos)
                        data.append(output)
                        break
        if len(data) > 0:
            return data
        else:
            return False
