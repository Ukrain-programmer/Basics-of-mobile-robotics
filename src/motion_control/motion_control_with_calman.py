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
        self.current_position = global_path[0]  # Start at the first waypoint
        self.current_orientation = 0  # Initial orientation (facing +X)
        # self.local_navigator = LocalNavigator(thymio_controller)
        self.speed = 150
        self.kalman_filter = KalmanFilter()
        self.x, self.y, self.theta = global_path[0][0], global_path[0][1], 0  # Initial state


    def move_to_target(self, target_position):
        """
        Move the robot to the target position, using Kalman filtering for state estimation.

        Args:
            target_position (tuple): The target (x, y) position.
        """
        print(f"Moving to target: {target_position}")
        target_x, target_y = target_position

        while True:
            # Get velocities
            left_speed, right_speed = self.thymio.get_speed()

            _, x, _, _ = self.kalman_filter.kalman_filter(False,[left_speed, right_speed], 0)
            self.x = x[0]
            self.y = x[1]
            self.theta = x[2]
            print(f"x: {self.x}, y: {self.y}, theta: {self.theta}")

            distance_to_target = ((self.x - target_x) ** 2 + (self.y - target_y) ** 2) ** 0.5

            print("distance_to_target: ", distance_to_target)
            if distance_to_target < 0.5:  # Threshold for reaching the waypoint
                print(f"Reached target: {target_position}")
                self.current_position = (self.x, self.y)
                self.thymio.stop()
                break

            self.thymio.set_speed(self.speed, self.speed)
            time.sleep(0.1)  # Sampling delay

    def execute_path(self):
        """
        Execute the path by moving through waypoints.
        """
        for i in range(len(self.path) - 1):
            target_position = self.path[i + 1]

            # Calculate target orientation
            target_orientation = self.calculate_target_orientation(target_position)

            # Align to target orientation
            self.align_to_target_orientation(target_orientation)

            # Move to the target position
            self.move_to_target(target_position)

        self.thymio.stop()
        print("Path execution complete.")

    def calculate_target_orientation(self, target_position):
        """
        Calculate the target orientation based on the next waypoint.
        """
        target_x, target_y = target_position
        dx = target_x - self.x
        dy = target_y - self.y
        angle = (math.atan2(dy, dx) * 180 / math.pi) % 360
        return angle

    def align_to_target_orientation(self, target_orientation):
        """
        Align the robot's orientation to the target orientation using turns.
        """
        angle_difference = (target_orientation - self.theta) % 360

        if angle_difference > 180:
            angle_difference -= 360

        print(f"Aligning to target orientation: {target_orientation}°")

        if abs(angle_difference) > 10:  # Allow small angle tolerance
            if angle_difference > 0:
                self.thymio.turn_left()
            else:
                self.thymio.turn_right()

            self.theta = (self.theta + angle_difference) % 360










if __name__ == "__main__":
    thymio = ThymioController()
    #path = [(23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]
    #path = [(26, 1), (25, 1), (24, 1), (23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]

    # Define a path as a list of (x, y) waypoints
    path = [(29, 16), (29, 17), (29, 18)]

    try:
        thymio.connect(timeout=5)

        # Pass the Kalman filter function to MotionControl
        motion_control = MotionControl(thymio, path, None)
        motion_control.execute_path()

    finally:
        thymio.disconnect()



