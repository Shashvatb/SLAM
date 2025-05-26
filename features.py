import math
import random
import numpy as np
from fractions import Fraction
from time import time
from scipy.odr import Model, RealData, ODR                    # orthogonal distance regression

landmarks = []

class featureDetection:
    def __init__(self):
        # vars
        self.epsilon = 2.5                                   # allowed distance between fitted line and predicted point
        self.delta = 7                                   # allowed distance between predicted and measured point
        self.s_num = 6                                       # number of points in our seed segment
        self.p_min = 7                                      # min number of points a seed segment should have
        self.g_max = 10                                       # how much distance between two points is considered a gap
        self.seed_segments = []
        self.line_segments = []
        self.laser_points = []
        self.line_params = None
        self.num_points = len(self.laser_points) - 1 
        self.l_min = 20                                      # min length of line segment
        self.l_real = 0                                      # real length of line segment
        self.l_laser_points = 0                              # legth of laser points in a line segment
        
        self.features = []

    # euclidean distance
    def get_distance_points(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))
    
    # get distance between a line (general form) and a point
    def get_distance_p2l(self, params, point):
        A, B, C = params
        distance = abs(A*point[0] + B*point[1] + C) / math.sqrt(A**2 + B**2)
        return distance
    
    # get two points from a line (slope intercept form)
    def get_line2points(self, m, b):
        x1, x2 = 2, 2000
        y1 = m * x1 + b
        y2 = m * x2 + b
        return [(x1, y1), (x2, y2)]
    
    # extract line that passes two points
    def get_points2line(self, p1, p2):
        m, b = 0, 0
        if p1[0] == p2[0]:
            pass
        else:
            m = (p2[1]-p1[1])/(p2[0]-p1[0])
            b = p2[1] - m * p2[0]
        return m, b 
    
    # convert line from general form to slope intercept form
    def get_general2si(self, params):
        A, B, C = params
        m = -A/B
        b = -C/B
        return m, b
    
    # convert line from slope intercept form to general form
    def get_si2general(self, m, b):
        A, B, C = -m, 1, -b
        if A < 0:
            A, B, C = -A, -B, -C
        denominator_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        denominator_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(denominator_a, denominator_c)  # greatest common denominator
        lcm = denominator_a * denominator_c / gcd   # lowest common multiple

        A = A * lcm
        B = B * lcm
        C = C * lcm
        return A, B, C
    
    # find intersection between two lines (general form) - assuming the lines intersect
    def find_intersection(self, param1, param2):
        A1, B1, C1 = param1
        A2, B2, C2 = param2
        x = (C1 * B2 - C2 * B1)/(B1 * A2 - B2 * A1)
        y = (A1 * C2 - C1 * A2)/(B1 * A2 - B2 * A1)
        return x,y
    
    # find orthogonal projection of the point on the line
    def project_point2line(self, p, m, b):
        x, y = p

        # Handle special cases
        if m == 0:  # horizontal line
            return x, b
        elif abs(m) > 1e6:  # near-vertical line
            return -b / m, y  # fallback in case m is huge, but not inf

        # Perpendicular line slope
        m_perp = -1 / m
        b_perp = y - m_perp * x

        # Solve for intersection
        proj_x = (b - b_perp) / (m - m_perp)
        proj_y = m * proj_x + b

        x_clamped = max(0, min(proj_x, 1200 - 1))
        y_clamped = max(0, min(proj_y, 600 - 1))

        return x_clamped, y_clamped
    
    # convert angle distance to cartesian position 
    def AD2pos(self, distance, angle, position):
        x = distance*math.cos(angle) + position[0]
        y = -distance*math.sin(angle) + position[1]
        return int(x), int(y)
    
    # store laser points from the data
    def laser_point_set(self, data):
        self.laser_points = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = self.AD2pos(point[0], point[1], point[2])
                self.laser_points.append([coordinates, point[1]])
                self.num_points = len(self.laser_points) - 1
        

    # fit a line to the laser points
    def linear_func(self, p, x):
        m, b = p
        return m*x + b
    
    # orthogonal fit -> we use this because it measures perpendicular distance from the points to the line
    def ord_fit(self, laser_points):
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points]) 
        
        # define model 
        model = Model(self.linear_func)          

        # create data for the model
        data = RealData(x, y)

        # create ord model
        odr_model = ODR(data, model, beta0=[0.,0.])

        # run regression
        out = odr_model.run()
        m, b = out.beta
        return m, b
        if out.info in [1, 2, 3, 4]:  # These are successful termination codes
            return m, b
        else:
            return float('nan'), float('nan')  # Failed to converge

    # predict points
    def predict_point(self, line_params, sensor_point, position):
        m, b = self.get_points2line(position, sensor_point)
        param1 = self.get_si2general(m, b)
        pred_x, pred_y = self.find_intersection(param1, line_params)
        return pred_x, pred_y
    
    # detect seed segment
    def detect_seed_segment(self, position, break_point_index):
        flag = True
        self.num_points = max(0, self.num_points)
        self.seed_segments = []
        for i in range(break_point_index, (self.num_points-self.p_min)):
            predicted_points_to_draw = []
            j = i + self.s_num   
            m, b = self.ord_fit(self.laser_points[i:j])
            if np.any(np.isnan((m, b))) or np.any(np.isinf((m, b))):
                break
            params = self.get_si2general(m, b)

            # check validity of seed points
            for k in range(i, j):
                predicted_point = self.predict_point(params, self.laser_points[k][0], position)
                predicted_points_to_draw.append(predicted_point)

                # distance between actual nad predicted point
                d1 = self.get_distance_points(predicted_point, self.laser_points[k][0])
                if d1 > self.delta:
                    flag = False
                    break

                d2 = self.get_distance_p2l(params=params, point=predicted_point)
                if d2 > self.epsilon:
                    flag = False
                    break
            if flag:
                self.line_params = params
                return [self.laser_points[i:j], predicted_points_to_draw, (i,j)]
        return False
    
    # grow seed segment
    def grow_seed_segment(self, indices, break_point):
        line_eq = self.line_params
        i, j = indices
        
        base_epsilon = self.epsilon  # base threshold
        tolerance = 1              # tweak this for sensitivity

        if self.laser_points is None or len(self.laser_points) == 0:
            self.laser_points = []  # Clear previous data to avoid stale use
            return

        point_beginning, point_finishing = break_point, min((j+1), len(self.laser_points)-1)

        # growing right
        while self.get_distance_p2l(line_eq, self.laser_points[point_finishing][0]) < self.epsilon:
            if point_finishing > self.num_points - 1:
                break
            else:
                m, b = self.ord_fit(self.laser_points[point_beginning:point_finishing])
                if np.any(np.isnan((m, b))) or np.any(np.isinf((m, b))):
                    break

                # Compute adaptive epsilon threshold
                angle = abs(np.arctan(m))
                epsilon_adj = base_epsilon * (1 + tolerance * np.sin(angle))

                dist = self.get_distance_p2l(line_eq, self.laser_points[point_finishing][0])
                if dist >= epsilon_adj:
                    break

                line_eq = self.get_si2general(m, b)
                point = self.laser_points[point_finishing][0]

            point_finishing = (point_finishing + 1) 
            next_point = self.laser_points[point_finishing][0]
            if self.get_distance_points(point, next_point) > self.g_max:           # could be any break (door, window for interior scene)
                break
        point_finishing = (point_finishing - 1) 

        # grow left
        while point_beginning > 0:
            next_point = self.laser_points[point_beginning - 1][0]
            current_point = self.laser_points[point_beginning][0]

            if self.get_distance_points(current_point, next_point) > self.g_max:
                break

            # Temporarily include the next point and fit the line
            temp_segment = self.laser_points[point_beginning - 1 : point_finishing]
            m, b = self.ord_fit(temp_segment)
            if np.any(np.isnan((m, b))) or np.any(np.isinf((m, b))):
                break

            # Compute adaptive epsilon threshold
            angle = abs(np.arctan(m))
            epsilon_adj = base_epsilon * (1 + tolerance * np.sin(angle))

            dist = self.get_distance_p2l(line_eq, self.laser_points[point_finishing][0])
            if dist >= epsilon_adj:
                break
            
            line_eq = self.get_si2general(m, b)
            if self.get_distance_p2l(line_eq, next_point) > self.epsilon:
                break

            point_beginning -= 1

        l_real = self.get_distance_points(self.laser_points[point_beginning][0], self.laser_points[point_finishing][0]) 
        l_laser_points = len(self.laser_points[point_beginning:point_finishing])

        if l_real >= self.l_min and l_laser_points >= self.p_min:
            self.line_params = line_eq
            m, b = self.get_general2si((line_eq[0], line_eq[1], line_eq[2]))
            self.two_points = self.get_line2points(m, b)
            self.line_segments.append((self.laser_points[point_beginning+1][0], self.laser_points[point_finishing-1][0]))
            return [
                self.laser_points[point_beginning:point_finishing],
                self.two_points,
                (self.laser_points[point_beginning+1][0], self.laser_points[point_finishing-1][0]),
                point_finishing,
                line_eq, 
                (m, b)
            ]
        else:
            return False
        

    # def grow_seed_segment(self, indices, break_point):
    #     line_eq = self.line_params
    #     i, j = indices

    #     # remove angle bias during detection
    #     if random.random() < 0.3:
    #         offset = random.randint(0, len(self.laser_points)-1)
    #         rotated = self.laser_points[offset:] + self.laser_points[:offset]
    #         self.laser_points = rotated

    #     # beginning point (PB) amd final point (PF)
    #     # NP = len(self.laser_points)
    #     point_beginning, point_finishing = max(break_point, (i-1)), min((j+1), len(self.laser_points)-1)
    #     # PB, PF = max(break_point, (i - 1) % NP), (j + 1) % NP

    #     # growing left
    #     while self.get_distance_p2l(line_eq, self.laser_points[point_finishing][0]) < self.epsilon:
    #         if point_finishing > self.num_points - 1:
    #             break
    #         else:
    #             m, b = self.ord_fit(self.laser_points[point_beginning:point_finishing])
    #             if np.any(np.isnan((m, b))) or np.any(np.isinf((m, b))):
    #                 break
    #             line_eq = self.get_si2general(m, b)
    #             point = self.laser_points[point_finishing][0]

    #         point_finishing = (point_finishing + 1) 
    #         next_point = self.laser_points[point_finishing][0]
    #         if self.get_distance_points(point, next_point) > self.g_max:           # could be any break (door, window for interior scene)
    #             break
    #     point_finishing = (point_finishing - 1) 

    #     # grow right
    #     while self.get_distance_p2l(line_eq, self.laser_points[point_beginning][0]) < self.epsilon:
    #         if point_beginning < break_point:
    #             break
    #         else:
    #             m, b = self.ord_fit(self.laser_points[point_beginning:point_finishing])
    #             if np.any(np.isnan((m, b))) or np.any(np.isinf((m, b))):
    #                 break
    #             line_eq = self.get_si2general(m, b)
    #             point = self.laser_points[point_beginning][0]
    #         point_beginning = (point_beginning + 1) 
    #         next_point = self.laser_points[point_beginning][0]
    #         if self.get_distance_points(point, next_point) > self.g_max:           # could be any break (door, window for interior scene)
    #             break
    #     point_beginning = (point_beginning + 1) 

    #     l_real = self.get_distance_points(self.laser_points[point_beginning][0], self.laser_points[point_finishing][0]) 
    #     l_laser_points = len(self.laser_points[point_beginning:point_finishing])

    #     if l_real >= self.l_min and l_laser_points >= self.p_min:
    #         self.line_params = line_eq
    #         m, b = self.get_general2si((line_eq[0], line_eq[1], line_eq[2]))
    #         self.two_points = self.get_line2points(m, b)
    #         self.line_segments.append((self.laser_points[point_beginning+1][0], self.laser_points[point_finishing-1][0]))
    #         return [
    #             self.laser_points[point_beginning:point_finishing],
    #             self.two_points,
    #             (self.laser_points[point_beginning+1][0], self.laser_points[point_finishing-1][0]),
    #             point_finishing,
    #             line_eq, 
    #             (m, b)
    #         ]
    #     else:
    #         return False

    def line_features2point(self):
        new_representation = []

        for feature in self.features:
            projection = self.project_point2line((0,0), feature[0][0], feature[0][1])
            new_representation.append([feature[0], feature[1], projection])
        
        return new_representation
    

    def landmark_association(self, landmark, threshold=10):  # threshold for matching landmarks (in pixels)
        for l in landmark:
            flag = False
            for l2_idx, l2 in enumerate(landmarks):
                dist = self.get_distance_points(l[2], l2[2])
                if dist < threshold:
                    if not self.is_overlap(l[1], l2[1]):
                        continue
                    else:
                        landmarks.pop(l2_idx)
                        landmarks.insert(l2_idx, l)
                        flag = True
                        break
            if not flag:
                landmarks.append(l)
    

    def is_overlap(self, seg1, seg2):
        length_1 = self.get_distance_points(seg1[0], seg1[1])
        length_2 = self.get_distance_points(seg2[0], seg2[1])
        center_1 = ((seg1[0][0] + seg1[1][0])/2, (seg1[0][1] + seg1[1][1])/2)
        center_2 = ((seg2[0][0] + seg2[1][0])/2, (seg2[0][1] + seg2[1][1])/2)
        dist = self.get_distance_points(center_1, center_2)
        if dist > (length_1 + length_2)/2:
            return False
        else:
            return True

