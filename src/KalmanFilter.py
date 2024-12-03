import numpy as np
import time
class KalmanFilter:
    def __init__(self, robot_length=0.011, speed_variance= 0.048755 ,conversion_factor=0.0387096):
        self.X = np.array([0, 0,0], dtype=float)
        self.U = np.array([0, 0], dtype=float)
        self.P = np.eye(3)
        self.Z = np.array([0, 0, 0], dtype=float)  # Measurement
        self.robot_length = robot_length
        self.speed_variance = speed_variance
        self.conversion_factor=conversion_factor
        self.theta=0

    def prediction_step(self, period):

        if period < 0.01:
            dt = 0.01 * self.conversion_factor/2
            time.sleep(0.01-period)
        else:
            dt = period * self.conversion_factor/2

        A = np.eye(3)
        theta = self.X[2]

        B = np.array([[0.5 * dt * np.cos(theta), 0.5 * dt * np.cos(theta)],
                      [-0.5 * dt * np.sin(theta), -0.5 * dt * np.sin(theta)],
                      [dt / self.robot_length, -dt / self.robot_length]])

        X_predicted = np.dot(A, self.X) + np.dot(B, self.U)

        Q = np.eye(2) * self.speed_variance
        P_predicted = self.P + np.dot(np.dot(B, Q), B.T)

        self.X = X_predicted
        self.P = P_predicted
        return self.X, self.P

    def update_step(self, Z, aruco_detected):

        # Measurement noise covariance
        R = np.diag([0.0006, 0.0003, 0.0002])

        # Measurement matrix
        H = np.eye(3) if aruco_detected else np.zeros((3, 3))

        # Innovation (difference between measurement and prediction)
        I = Z - np.dot(H, self.X)

        # Innovation covariance
        S = np.dot(np.dot(H, self.P), H.T) + R

        # Kalman gain
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # State update
        self.X = self.X + np.dot(K, I)

        # Covariance update
        self.P = np.dot(np.eye(3) - np.dot(K, H), self.P)

        return self.X, self.P

    def kalman_filter(self, aruco_detected, U, Z, period):
        self.U = U  # Update control input

        # Prediction step
        X_predicted, P_predicted = self.prediction_step(period)

        # Update step
        if aruco_detected:
            self.X, self.P = self.update_step(Z, aruco_detected)

        return self.X, self.P, X_predicted
