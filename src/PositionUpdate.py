import numpy as np
import cv2
import time
from KalmanFilter import KalmanFilter
from Image_EKF import Image_EKF


class PositionUpdate:
    def __init__(self, thymio):
        self.odom_position = []
        self.current_time = time.time()
        self.period = 0
        self.yaw_odom = 0
        self.yaw_aruco = 0
        self.marker_centers = []
        self.odom_centers = []
        self.initial = False
        self.kidnapped = False
        self.odom_tvec = np.array([[0.], [0.], [0.]])
        self.odom_rvec = np.array([[0.], [0.], [0.]])
        self.initial_tz = 0
        self.marker_corners_3d = 0
        self.kf = KalmanFilter()
        self.thymio = thymio

    @staticmethod
    def draw_trajectory(img, odom_position, color=(0, 255, 255), thickness=10):
        for k in range(1, len(odom_position)):
            start_point = tuple(odom_position[k - 1][:2])
            end_point = tuple(odom_position[k][:2])
            cv2.line(img, start_point, end_point, color, thickness)

    @staticmethod
    def draw_path(image, marker_centers, color=(255, 0, 0), thickness=2):
        if len(marker_centers) > 1:
            for j in range(1, len(marker_centers)):
                cv2.line(image, tuple(marker_centers[j - 1]), tuple(marker_centers[j]), color, thickness)

    def update_odom_position(self, marker_corners_3d, newcameramtx, dist, period, centers, initial,
                             aruco_detected, U, Z):

        odom_rvec = np.array([[0.], [0.], [0.]])
        X_updated, P_updated, X_predicted = self.kf.kalman_filter(aruco_detected, U, Z, period)
        X_updated = np.round(X_predicted, 5)
        yaw = X_updated[2]
        odom_tvec = np.array([[X_updated[0]], [X_updated[1]], np.array([initial])], dtype=np.float64).reshape(3, 1)

        # Convert odometry vectors to the appropriate format
        odom_tvec = np.array(odom_tvec, dtype=np.float64).reshape(3, 1)
        odom_rvec = np.array(odom_rvec, dtype=np.float64).reshape(3, 1)
        projected_points_odom, _ = cv2.projectPoints(marker_corners_3d, odom_rvec, odom_tvec, newcameramtx, dist)
        projected_points_odom = projected_points_odom.reshape(-1, 2).astype(int)

        # Compute the center of the projected points
        center_odom = np.mean(projected_points_odom, axis=0).astype(int)
        centers.append(center_odom)
        return odom_tvec, odom_rvec, center_odom, period, centers, yaw

    @staticmethod
    def euler_from_rvec(rvec):
        R, _ = cv2.Rodrigues(rvec)

        # Calculate Euler angles (roll, pitch, yaw)
        pitch = np.arctan2(R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        if yaw > -180 and yaw < 0:
            yaw = abs(yaw)
        else:
            yaw = 360 - yaw

        return roll, pitch, yaw

    def path_predicition(self, newcameramtx, dist, aruco_dict, aruco_params, mtx, img):
        self.current_time = time.time()
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Undistort the image
        undistorted = cv2.undistort(img_gray, mtx, dist, None, newcameramtx)
        corners, marker_id, robot_detected = Image_EKF.detect_marker_with_id(undistorted, aruco_dict, aruco_params,
                                                                             target_id=5)
        if robot_detected:

            cv2.aruco.drawDetectedMarkers(img, [corners], np.array([marker_id]), (0, 255, 0))
            center, projected_points, tvec_marker, rvec_marker, self.marker_centers, marker_corners_3d = Image_EKF.process_marker(
                corners, marker_id, 5, newcameramtx, dist, self.marker_centers)
            self.kf.Z[0] = tvec_marker[0][0]
            self.kf.Z[1] = tvec_marker[1][0]
            self.kf.Z[2] = np.radians(PositionUpdate.euler_from_rvec(rvec_marker)[2])

            if not self.initial or self.kidnapped:
                self.odom_tvec = tvec_marker
                self.odom_centers.append(center)
                self.initial_tz = self.odom_tvec[2][0]
                self.kf.X = self.kf.Z
                # kf.X[2]=0
                self.kf.U = self.thymio.get_speed()

                self.odom_tvec, self.odom_rvec, center_odom, self.period, self.odom_centers, self.yaw_odom = self.update_odom_position(
                    self.marker_corners_3d, newcameramtx, dist, self.period, self.odom_centers,
                    self.initial_tz, False, self.kf.U, self.kf.Z)
                self.initial = True
                self.kf.theta = 0
                end_time = time.time()
                self.period = end_time - self.current_time
        else:
            print("robot not found.")
            self.kf.U = self.thymio.get_speed()
            self.odom_tvec, self.odom_rvec, center_odom, self.period, self.odom_centers, self.yaw_odom = self.update_odom_position(
                self.marker_corners_3d, newcameramtx, dist, self.period, self.odom_centers,
                self.initial_tz, True, self.kf.U, self.kf.Z)
            self.kf.theta = self.yaw_odom
            end_time = time.time()
            self.period = end_time - self.current_time
        if self.initial:
            self.kf.U = self.thymio.get_speed()
            # current_time=time.time()
            self.odom_tvec, self.odom_rvec, center_odom, self.period, self.odom_centers, self.yaw_odom = self.update_odom_position(
                self.marker_corners_3d, newcameramtx, dist, self.period, self.odom_centers,
                self.initial_tz, True, self.kf.U, self.kf.Z)
            self.kf.theta = self.yaw_odom
            end_time = time.time()
            self.period = end_time - self.current_time
