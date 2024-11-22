import math
import time
from ThymioController import ThymioController


class MotionControl:
    def __init__(self, thymio_controller, global_path):
        """
        Initialize the MotionControl class.

        Args:
            thymio_controller (ThymioController): An instance of ThymioController.
            global_path (list of tuples): The global navigation path as a list of (x, y) waypoints.
        """
        self.thymio = thymio_controller
        self.path = global_path
        self.current_index = 0
        self.current_position = global_path[0]
        self.current_angle = 0  # Initial angle is aligned with +X-axis
        self.movement_threshold = 0.05  # Threshold for reaching waypoints

    def calculate_angle(self, target_position):
        """
        Calculate the angle to align with the target position,
        ensuring rotations are limited to 90-degree increments.

        Args:
            target_position (tuple): The (x, y) coordinates of the target waypoint.

        Returns:
            float: The target angle (in radians) for the next movement.
        """
        x1, y1 = self.current_position
        x2, y2 = target_position

        if x2 > x1:  # Moving right (+X)
            return 0
        elif x2 < x1:  # Moving left (-X)
            return math.pi
        elif y2 > y1:  # Moving up (+Y)
            return math.pi / 2
        elif y2 < y1:  # Moving down (-Y)
            return -math.pi / 2
        else:
            return self.current_angle  # No change in position


    def rotate_to_angle(self, target_angle):
        """
        Rotate the robot to align with the target angle in 90-degree steps.

        Args:
            target_angle (float): The desired angle (in radians).
        """
        angle_difference = target_angle - self.current_angle

        # Normalize the angle to [-pi, pi]
        angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi

        # Ensure the angle difference corresponds to ±90 or ±270 degrees
        if abs(angle_difference) in [math.pi / 2, 3 * math.pi / 2]:
            rotation_speed = 100 if angle_difference > 0 else -100

            # Experimentally determine the time needed for a 90-degree rotation
            rotation_duration = 1.0  # Example duration; adjust for your Thymio robot
            self.thymio.set_speed(-rotation_speed, rotation_speed)
            time.sleep(rotation_duration)
            self.thymio.stop()

            # Update the robot's current angle
            self.current_angle = target_angle

    def move_to_target(self, target_position):
        """
        Move the robot straight to the target position.

        Args:
            target_position (tuple): The target (x, y) position.
        """
        x1, y1 = self.current_position
        x2, y2 = target_position
        self.thymio.set_speed(150, 150)
        time.sleep(0.5)
        # distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        # while distance > self.movement_threshold:
        #     self.thymio.set_speed(150, 150)
        #     time.sleep(2)  # Move in small steps
        #     # Simulate updating current position (replace with actual localization)
        #     # x1 += (x2 - x1) * 0.1
        #     # y1 += (y2 - y1) * 0.1
        #     # distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # self.thymio.stop()
        self.current_position = target_position

    def execute_path(self):
        """
        Execute the path by moving through waypoints.
        """
        for i in range(len(self.path) - 1):
            target_position = self.path[i + 1]

            # Calculate the target angle and rotate
            target_angle = self.calculate_angle(target_position)
            self.rotate_to_angle(target_angle)

            # Move to the target position
            self.move_to_target(target_position)

            print(f"Reached waypoint: {target_position}")

        print("Path execution complete.")








if __name__ == "__main__":
    thymio = ThymioController()
    #path = [(23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]
    #path = [(26, 1), (25, 1), (24, 1), (23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]
    path = [(19, 1), (20, 1), (21, 1), (21, 2), (21, 3), (22, 3)]

    try:
        thymio.connect(timeout=5)
        motion_control = MotionControl(thymio, path)
        motion_control.execute_path()
    finally:
        thymio.disconnect()


