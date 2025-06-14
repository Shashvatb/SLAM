import numpy as np

# Kalman Filter
class KalmanFilter:
    def __init__(self, w=800, h=600):
        # noise
        self.Q = np.diag([0.5, 0.5]) ** 2   # Process noise (predict)
        self.R = np.diag([1.0, 1.0]) ** 2   # Measurement noise (observe)
        self.P = np.diag([5.0, 5.0])        # state covariance   

        self.x_est = np.array([w/2.0, h/2.0])  # initial state (center)
     

        # matrices
        self.A = np.eye(2)   # State transition matrix
        self.B = np.eye(2)   # Control matrix
        self.C = np.eye(2)   # Observation matrix

    def predict(self, x_est, u):
        x_pred = self.A @ x_est + self.B @ u                # predicted x with the linear model
        P_pred = self.A @ self.P @ self.A.T + self.Q        # predicted covariance (with process noise)
        return x_pred, P_pred


    def observe(self, x_true):
        z = x_true + np.random.multivariate_normal([0, 0], self.R)  # nosiy observation (with measurement noise)
        return z 

    
    def update(self, x_pred, P_pred, z):
        S = self.C @ P_pred @ self.C.T + self.R        # Innovation covariance
        K = P_pred @ self.C.T @ np.linalg.inv(S)       # Kalman gain -> how much i trust my observation?

        y = z - self.C @ x_pred                        # Innovation

        self.x_est = x_pred + K @ y                         # updated state
        self.P = (np.eye(2) - K @ self.C) @ P_pred     # updated covariance 


# Extended Kalman Filter -> local linearization for non linear functions using taylor expansion (with the help of jacobian - first order derivative)
# how far are we from the actual scenario? It depends on two things, how non linear the function is (how far is the linear finction from the original) 
# and how much initial uncertainty we have (state covariance)
class ExtendedKalmanFilter:
    def __init__(self, w=800, h=600, theta=0.0):
        # State: [x, y, theta]
        self.x_est = np.array([w/2.0, h/2.0, theta])      # Initial state estimate

        # Covariances
        self.P = np.diag([5.0, 5.0, np.deg2rad(10)])    # state covariance
        self.Q = np.diag([0.5, 0.5, np.deg2rad(5)])**2  # Process noise (predict)
        self.R = np.diag([1.0, np.deg2rad(5)])**2       # Measurement noise (observe)

    # Normalize angle to [-π, π]
    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    # Nonlinear motion model
    def f(self, x, u):
        x_, y_, theta_ = x
        dx, dtheta = u
        theta_new = theta_ + dtheta
        x_new = x_ + dx * np.cos(theta_new)
        y_new = y_ + dx * np.sin(theta_new)
        return np.array([x_new, y_new, theta_new])

    # Jacobian of the motion model wrt state
    def G(self, x, u):
        _, _, theta = x
        dx, dtheta = u
        theta_new = theta + dtheta
        return np.array([
            [1, 0, -dx * np.sin(theta_new)],
            [0, 1,  dx * np.cos(theta_new)],
            [0, 0, 1]
        ])

    # Nonlinear observation model
    def h(self, x):
        x_, y_, theta_ = x
        range_ = np.sqrt(x_**2 + y_**2)
        bearing = np.arctan2(y_, x_) - theta_
        return np.array([range_, self.normalize_angle(bearing)])

    # Jacobian of the observation model wrt state
    def H(self, x):
        x_, y_, _ = x
        q = x_**2 + y_**2
        sqrt_q = np.sqrt(q)
        return np.array([
            [x_ / sqrt_q, y_ / sqrt_q, 0],
            [-y_ / q,      x_ / q,     -1]
        ])

    # observe (with noise introduced)
    def observe(self, x_true):
        z_true = self.h(x_true)
        noise = np.random.multivariate_normal([0, 0], self.R)
        return z_true + noise

    def predict(self, u):
        x_pred = self.f(self.x_est, u)
        G_t = self.G(self.x_est, u)
        self.P = G_t @ self.P @ G_t.T + self.Q
        self.x_est = x_pred

    def update(self, z):
        H_t = self.H(self.x_est)
        z_pred = self.h(self.x_est)
        y = z - z_pred
        y[1] = self.normalize_angle(y[1])
        S = H_t @ self.P @ H_t.T + self.R
        K = self.P @ H_t.T @ np.linalg.inv(S)
        self.x_est = self.x_est + K @ y
        self.P = (np.eye(3) - K @ H_t) @ self.P

class ParticleFilter:
    def __init__(self):
        pass
    