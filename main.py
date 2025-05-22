import math
import pygame

import env
import sensors

if __name__ == '__main__':
    environment = env.buildEnvironment((600, 1200))
    environment.original_map = environment.map.copy()
    laser = sensors.laserSensor(200, environment.original_map, uncertainty=(0.5,0.01))
    environment.map.fill(environment.black)
    environment.info_map = environment.map.copy()
    running = True
    while running:
        sensorOn = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if pygame.mouse.get_focused():
                sensorOn = True
            elif not pygame.mouse.get_focused():
                sensorOn = False
        if sensorOn:
            position = pygame.mouse.get_pos()
            laser.pos = position
            sensor_data = laser.sense_obstacles()
            environment.data_storage(sensor_data)
            environment.show_sensor_data()
        environment.map.blit(environment.info_map, (0,0))
        pygame.display.update()
    pygame.quit()