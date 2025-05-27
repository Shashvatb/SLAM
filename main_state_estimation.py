import numpy as np

import pygame
import argparse
from filters import KalmanFilter


if __name__ == "__main__":
    pygame.init()
    w, h = 800, 600
    screen = pygame.display.set_mode((w, h))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 16)

    # initialize filter
    filter = KalmanFilter()
    x_est = np.array([w/2.0, h/2.0])  # initial state (center)
    x_true = x_est.copy()

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
        u = mouse_pos - x_true                                       # control
        x_true = mouse_pos   

        # predict
        x_pred, P_pred = filter.predict(x_true, u)

        # observe
        z = filter.observe(x_true)           
        
        # update
        x_est = filter.update(x_pred, P_pred, z)

        # add point to the path
        true_path.append(x_true.copy())
        measured_path.append(z.copy())
        estimated_path.append(x_est.copy())

        # Draw
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