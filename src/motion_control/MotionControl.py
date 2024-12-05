import numpy as np
from src.Constants import DISTANCE_THRESHOLD, WAYPOINT_STEP_SIZE, ANGLE_THRESHOLD
from src.motion_control.ThymioController import ThymioController


class MotionControl:
    def __init__(self, thymio_controller, global_path):
        """
        Initialize the motion control system for the robot.

        Args:
            thymio_controller (ThymioController): An instance of the ThymioController class to control the robot's motors.
            global_path (list): A list of waypoints representing the robot's path.
        """
        self.thymio = thymio_controller
        self.path = global_path
        self.path_iterator = 12  # First step
        self.base_speed = 40
        self.current_x, self.current_y, self.current_theta = 0, 0, 0

    def setup_position(self, current_pos):
        """
        Setup the robot's current position and orientation.

        Args:
            current_pos (tuple): A tuple containing the current x, y, and theta (orientation) values.
        """
        self.current_x, self.current_y, self.current_theta = current_pos[0], current_pos[1], current_pos[2]

    def setup_new_path(self, path, theta):
        """
        Set a new path for the robot to follow and reset the path iterator.

        Args:
            path (list): A new list of waypoints for the robot to follow.
            theta (float): The new orientation (theta) of the robot.
        """
        self.path = path
        self.path_iterator = 0
        self.current_x, self.current_y, self.current_theta = path[0][0], path[0][1], theta

    def move_to_target(self, target_position, current_position: list):
        """
        Move the robot towards the target position by calculating the distance and adjusting its speed and orientation.

        Args:
            target_position (tuple): The x and y coordinates of the target position.
            current_position (list): The current position and orientation of the robot (x, y, theta).
        """
        target_x, target_y = target_position
        self.current_x, self.current_y = current_position[0], current_position[1]
        self.current_theta = current_position[2]

        # Calculate distance to target
        distance_to_target = np.sqrt((self.current_x - target_x) ** 2 + (self.current_y - target_y) ** 2)

        if distance_to_target < DISTANCE_THRESHOLD:
            self.path_iterator += WAYPOINT_STEP_SIZE
            return

        # Check if the robot is close to the last point in the path
        if np.sqrt((self.current_x - self.path[-1][0]) ** 2 + (self.current_y - self.path[-1][1]) ** 2) < 120:
            self.path_iterator += 1000

        # Calculate the angle to the target
        angle_to_target = np.arctan2(target_y - self.current_y, target_x - self.current_x)
        angle_difference = angle_to_target - self.current_theta

        # Normalize angle difference to [-pi, pi]
        angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi

        if abs(angle_difference) > ANGLE_THRESHOLD:
            turn_factor = angle_difference / np.pi  # Normalized factor between 0 and 1
            # Turn left
            if angle_difference > -2.5:
                left_speed = self.base_speed + (60 * turn_factor)
                right_speed = self.base_speed - (40 * turn_factor)

                self.thymio.set_speed(int(left_speed), int(right_speed))
            else:
                # Move straight to target
                self.thymio.set_speed(self.base_speed, self.base_speed)
