import random
import pygame

import env
import features
import sensors

# random colors for predicted points
def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

if __name__ == '__main__':
    feature_map = features.featureDetection()
    environment = env.buildEnvironment((600, 1200))
    environment.original_map = environment.map.copy()
    laser = sensors.laserSensor(200, environment.original_map, uncertainty=(0.5,0.01))
    environment.map.fill(environment.white)
    environment.info_map = environment.map.copy()
    running = True
    feature_detection = True
    break_point_index = 0
    last_mouse_position = None

    while running:
        environment.info_map = environment.map.copy()
        feature_detection = True
        end_points = [0, 0]
        sensorOn = False
        predicted_points_to_draw = []

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if pygame.mouse.get_focused():
            sensorOn = True
        elif not pygame.mouse.get_focused():
            sensorOn = False
        if sensorOn:
            position = pygame.mouse.get_pos()
            if position != last_mouse_position:
                last_mouse_position = position
                break_point_index = 0
            laser.pos = position
            sensor_data = laser.sense_obstacles()
            feature_map.laser_point_set(sensor_data)
            while break_point_index < (feature_map.num_points - feature_map.p_min):
                seed_segment = feature_map.detect_seed_segment(laser.pos, break_point_index)
                if seed_segment == False:
                    break
                else:
                    seed_segment, predicted_points_to_draw, indices = seed_segment[0], seed_segment[1], seed_segment[2]
                    result = feature_map.grow_seed_segment(indices, break_point_index)
                    if result == False:
                        break_point_index = indices[1]
                        # break_point_index += 1
                        continue
                    else:
                        line_seg = result[0]
                        line_eq = result[1]
                        outermost_points = result[2]
                        break_point_index = result[3]
                        m, b = result[5]
                        feature_map.features.append([[m,b], outermost_points])
                        environment.data_storage(sensor_data)

                        feature_map.features = feature_map.line_features2point()
                        feature_map.landmark_association(feature_map.features)
            
            landmarks = features.line_segment_nms([i[1] for i in features.landmarks], angle_thresh=2, dist_thresh=3)
            for l in landmarks:
                pygame.draw.line(environment.info_map, environment.blue, l[0], l[1], 2)
        environment.map.blit(environment.info_map, (0,0))
        pygame.display.update()
    print("number of landmarks: ", len(landmarks), " for example: ", landmarks[0], " features: ", feature_map.features[0])

