import math
import time
from ThymioController import ThymioController
from src.KalmanFilter import KalmanFilter


class MotionControl:
    def __init__(self, thymio_controller, global_path, kalman_filter: KalmanFilter):
        self.thymio = thymio_controller
        self.path = global_path
        self.path_iterator = 0
        # self.current_position = global_path[0]  # Start at the first waypoint
        self.current_orientation = 0  # Initial orientation (facing +X)
        self.speed = 50
        self.kalman_filter = KalmanFilter()
        self.current_x, self.current_y, self.current_theta = global_path[0][0], global_path[0][1], 0  # Initial state

    def setup_new_path(self, path, theta):
        self.path = path
        self.path_iterator = 0
        self.current_x, self.current_y, self.current_theta = path[0][0], path[0][1], theta

    def move_to_target(self, target_position):
        print(f"Moving to target: {target_position}")
        target_x, target_y = target_position

        left_speed, right_speed = self.thymio.get_speed()

        _, x, _, _ = self.kalman_filter.kalman_filter(False, [left_speed, right_speed], 0)

        self.current_x = x[0]
        self.current_y = x[1]
        self.current_theta = x[2]

        if self.current_x == target_x and target_y == self.current_y:
            # self.current_position = (self.current_x, self.current_y)
            self.path_iterator += 1
            return

        distance_to_target = ((self.current_x - target_x) ** 2 + (self.current_y - target_y) ** 2) ** 0.5

        print(f"Predicted x: {self.current_x}, y: {self.current_y}\n distance_to_target: ", distance_to_target)
        if distance_to_target < 0.5:  # Threshold for reaching the waypoint
            print(f"Reached target: {target_position}")
            # self.current_position = (self.current_x, self.current_y)
            self.path_iterator += 1
            return

        self.thymio.set_speed(self.speed, self.speed)

    def calculate_target_orientation(self, target_position):
        x1, y1 = self.current_x, self.current_y
        x2, y2 = target_position

        if x2 < x1:  # Moving right (+X)
            return 0
        elif x2 > x1:  # Moving left (-X)
            return 180
        elif y2 > y1:  # Moving up (+Y)
            return 90
        elif y2 < y1:  # Moving down (-Y)
            return 270

    def align_to_target_orientation(self, target_orientation):
        if target_orientation is None:
            return
        angle_difference = (target_orientation - self.current_orientation) % 360

        if angle_difference == 90:  # Turn right
            self.thymio.turn_right()
            self.current_orientation = (self.current_orientation + 90) % 360
        elif angle_difference == 270:  # Turn left
            self.thymio.turn_left()
            self.current_orientation = (self.current_orientation - 90) % 360
        elif angle_difference == 180:
            print(f"Unexpected 180-degree rotation required.")
        else:
            print("Already facing the correct direction.")


if __name__ == "__main__":
    thymio = ThymioController()
    # Define a path as a list of (x, y) waypoints
    path = [(29, 16), (29, 17), (29, 18)]

    try:
        thymio.connect(timeout=5)

        # Pass the Kalman filter function to MotionControl
        motion_control = MotionControl(thymio, path, None)

        while True:
            target_position = motion_control.path[motion_control.path_iterator]
            target_orientation = motion_control.calculate_target_orientation(target_position)
            motion_control.align_to_target_orientation(target_orientation)
            motion_control.move_to_target(target_position)
            time.sleep(0.5)

    finally:
        thymio.disconnect()
