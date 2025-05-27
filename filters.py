import numpy as np

# Kalman Filter
class KalmanFilter:
    def __init__(self):
        # noise
        self.Q = np.diag([0.5, 0.5]) ** 2   # Process noise (predict)
        self.R = np.diag([1.0, 1.0]) ** 2   # Measurement noise (observe)
        self.P = np.diag([5.0, 5.0])        # state covariance        

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

        x_est = x_pred + K @ y                         # updated state
        self.P = (np.eye(2) - K @ self.C) @ P_pred     # updated covariance 
        return x_est


# Extended Kalman Filter
