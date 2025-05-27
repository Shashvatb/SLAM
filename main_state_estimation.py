import numpy as np

import pygame
import argparse
from filters import KalmanFilter, ExtendedKalmanFilter, ParticleFilter


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--filter', help='choose between Kalman Filter (KF), Extended Kalman Filter (EKF) or Partical Filter (PF)')
    args = parser.parse_args()
    filter_name = args.filter

    pygame.init()
    w, h = 800, 600
    screen = pygame.display.set_mode((w, h))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 16)

    # initialize filter
    filter = None
    if filter_name.lower() == "kf":
        filter = KalmanFilter()
    elif filter_name.lower() == "ekf":
        filter = ExtendedKalmanFilter()
    elif filter_name.lower() == "pf":
        filter = ParticleFilter()
    x_true = filter.x_est.copy()

    # initialize path
    true_path = []
    measured_path = []
    estimated_path = []
    
    running = True
    while running:
        screen.fill((255, 255, 255))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        mouse_pos = np.array(pygame.mouse.get_pos(), dtype=float)    # original position

        if filter_name.lower() == "kf": 
            u = mouse_pos - x_true                                       # control
            x_true = mouse_pos   

            # predict, observe, update
            x_pred, P_pred = filter.predict(x_true, u)
            z = filter.observe(x_true)           
            filter.update(x_pred, P_pred, z)

            # add point to the path
            true_path.append(x_true.copy())
            measured_path.append(z.copy())
            estimated_path.append(filter.x_est.copy())

        elif filter_name.lower() == "ekf": 
            dx = mouse_pos - x_true[:2]
            distance = np.linalg.norm(dx)
            dtheta = 0.0  # For simplicity
            u = np.array([distance, dtheta])
            x_true[:2] = mouse_pos

            # Predict, observe, update
            filter.predict(u)
            z = filter.observe(x_true)
            filter.update(z)

            # For visualization
            true_path.append(x_true[:2].copy())
            range_meas, bearing_meas = z
            theta_est = filter.x_est[2]
            meas_x = filter.x_est[0] + range_meas * np.cos(theta_est + bearing_meas)
            meas_y = filter.x_est[1] + range_meas * np.sin(theta_est + bearing_meas)
            measured_path.append(np.array([meas_x, meas_y]))
            estimated_path.append(filter.x_est[:2].copy())
            true_path.append(x_true[:2].copy())

        # Draw
        # Draw paths as connected lines
        if len(true_path) > 1:
            pygame.draw.lines(screen, (0, 255, 0), False, [pt.astype(int) for pt in true_path], 1)
        if len(measured_path) > 1:
            pygame.draw.lines(screen, (255, 0, 0), False, [pt.astype(int) for pt in measured_path], 1)
        if len(estimated_path) > 1:
            pygame.draw.lines(screen, (0, 0, 255), False, [pt.astype(int) for pt in estimated_path], 1)

        # draw circles on each point for clarity
        for pt in true_path:
            pygame.draw.circle(screen, (0, 255, 0), pt.astype(int), 2)
        for pt in measured_path:
            pygame.draw.circle(screen, (255, 0, 0), pt.astype(int), 2)
        for pt in estimated_path:
            pygame.draw.circle(screen, (0, 0, 255), pt.astype(int), 2)
        screen.blit(font.render("Green: True | Red: Measured | Blue: Estimated", True, (0, 0, 0)), (10, 10))
        pygame.display.flip()
        clock.tick(10)

    pygame.quit()