import math
import time
from ThymioController import ThymioController
from src.kalman_filter import KalmanFilter


class MotionControl:
    def __init__(self, thymio_controller, global_path, kalman_filter):
        """
        Initialize the MotionControl class.

        Args:
            thymio_controller (ThymioController): An instance of ThymioController.
            global_path (list of tuples): The global navigation path as a list of (x, y) waypoints.
            kalman_filter (function): The Kalman filter function for state estimation.
        """
        self.thymio = thymio_controller
        self.path = global_path
        self.path_iterator = 0
        # self.current_position = global_path[0]  # Start at the first waypoint
        self.current_orientation = 0  # Initial orientation (facing +X)
        # self.local_navigator = LocalNavigator(thymio_controller)
        self.speed = 50
        self.kalman_filter = KalmanFilter()
        self.current_x, self.current_y, self.current_theta = global_path[0][0], global_path[0][1], 0  # Initial state


    def move_to_target(self, target_position):
        """
        Move the robot to the target position, using Kalman filtering for state estimation.

        Args:
            target_position (tuple): The target (x, y) position.
        """
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


    def kalman_mok(self):
        if self.mok >= 3:
            return [29, 17, 0]
        else:
            self.mok+=1
            return [29, 16, 0]

    def calculate_target_orientation(self, target_position):
        """
        Calculate the target orientation based on the next waypoint.
        """
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
        """
        Align the robot's orientation to the target orientation using 90-degree turns.
        """

        if target_orientation == None:
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
    # path = [(23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]
    # path = [(26, 1), (25, 1), (24, 1), (23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]

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

