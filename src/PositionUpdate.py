import numpy as np
import cv2

import KalmanFilter


class PositionUpdate:
    def __init__(self, kalman_filter: KalmanFilter):
        pass
        self.kl = kalman_filter

    @staticmethod
    def draw_trajectory(img, odom_position, color=(0, 255, 255), thickness=10):
        for k in range(1, len(odom_position)):
            start_point = tuple(odom_position[k-1][:2])  # First two elements of the previous position
            end_point = tuple(odom_position[k][:2])     # First two elements of the current position
            cv2.line(img, start_point, end_point, color, thickness)

    @staticmethod
    def draw_path(image, marker_centers, color=(255, 0, 0), thickness=2):
        if len(marker_centers) > 1:
            for j in range(1, len(marker_centers)):
                cv2.line(image, tuple(marker_centers[j-1]), tuple(marker_centers[j]), color, thickness)

    def update_odom_position(self, odom_tvec, marker_corners_3d, newcameramtx, dist, period,centers,yaw,initial,aruco_detected,U,Z):

        odom_rvec=  np.array([[0.], [0.], [0.]])
        X_updated, P_updated, X_predicted = self.kf.kalman_filter(aruco_detected, U, Z, period)
        X_updated=np.round(X_predicted,5)
        yaw=X_updated[2]
        odom_tvec=np.array([[X_updated[0]],[X_updated[1]],np.array([ initial])], dtype=np.float64).reshape(3, 1)

        # Convert odometry vectors to the appropriate format
        odom_tvec = np.array(odom_tvec, dtype=np.float64).reshape(3, 1)
        odom_rvec = np.array(odom_rvec, dtype=np.float64).reshape(3, 1)
        projected_points_odom, _ = cv2.projectPoints(marker_corners_3d,odom_rvec,odom_tvec,newcameramtx, dist)
        projected_points_odom = projected_points_odom.reshape(-1, 2).astype(int)

        # Compute the center of the projected points
        center_odom = np.mean(projected_points_odom, axis=0).astype(int)
        centers.append(center_odom)
        return odom_tvec, odom_rvec, center_odom, period,centers,yaw

    @staticmethod
    def euler_from_rvec(rvec):
        R, _ = cv2.Rodrigues(rvec)

        # Calculate Euler angles (roll, pitch, yaw)
        pitch = np.arctan2(R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        if yaw >-180 and yaw < 0:
            yaw=abs(yaw)
        else:
            yaw=360-yaw

        return roll, pitch, yaw

