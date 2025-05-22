import math
import pygame


class buildEnvironment:
    def __init__(self, map_dims):
        # initialize variables
        self.point_cloud = []
        self.external_map = pygame.image.load('map.png')
        self.map_height, self.map_width = map_dims
        self.window_name = 'Lidar SLAM'
        
        # define colors
        self.black = (0, 0, 0)
        self.gray = (70, 70, 70)
        self.red = (255, 0, 0)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.white = (255, 255, 255)

        # create pygame window
        pygame.init()
        pygame.display.set_caption(self.window_name)
        self.map = pygame.display.set_mode((self.map_width, self.map_height))
        self.map.fill(self.white)
        self.map.blit(self.external_map, (0,0))

    def AD2pos(self, distance, angle, position):
        x = distance*math.cos(angle) + position[0]
        y = -distance*math.sin(angle) + position[1]
        return int(x), int(y)

    def data_storage(self, data):
        for element in data:
            point = self.AD2pos(element[0], element[1], element[2])
            if point not in self.point_cloud:
                self.point_cloud.append(point)
    
    def show_sensor_data(self):
        self.info_map = self.map.copy()
        print(len(self.point_cloud))
        for point in self.point_cloud:
            self.info_map.set_at((int(point[0]), int(point[1])), self.red)
