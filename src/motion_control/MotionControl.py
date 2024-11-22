import math
import time
from ThymioController import ThymioController


class MotionControl:
    def __init__(self, thymio_controller, path):
        """
        Initialize the MotionControl with a ThymioController instance and a path.

        Args:
            thymio_controller (ThymioController): Instance of ThymioController.
            path (list of tuples): List of waypoints [(x1, y1), (x2, y2), ...].
        """
        self.thymio = thymio_controller
        self.path = path  # Global navigation path
        self.current_target_index = 0  # Index of the current target in the path
        self.current_position = path[0]  # Initial position (assume it's the first coordinate of the path)
        self.lookahead_distance = 0.1  # Distance to look ahead for path following (adjust as needed)
        self.speed = 100  # Base speed for the robot (adjustable)
        self.current_axis = None  # Current axis of movement ('x' or 'y')

    def compute_distance(self, pos1, pos2):
        """
        Compute the Euclidean distance between two points.

        Args:
            pos1 (tuple): First point (x1, y1).
            pos2 (tuple): Second point (x2, y2).

        Returns:
            float: Euclidean distance.
        """
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

    def determine_axis(self, current_pos, target_pos):
        """
        Determine the axis of movement between two points.

        Args:
            current_pos (tuple): Current position (x, y).
            target_pos (tuple): Target position (x, y).

        Returns:
            str: 'x' if moving along x-axis, 'y' if moving along y-axis, None if no movement.
        """
        if current_pos[0] != target_pos[0]:
            return 'x'  # Movement along x-axis
        elif current_pos[1] != target_pos[1]:
            return 'y'  # Movement along y-axis
        return None  # No movement

    def rotate_to_new_axis(self, new_axis):
        """
        Rotate the robot to align with a new axis.

        Args:
            new_axis (str): The new axis of movement ('x' or 'y').
        """
        if self.current_axis is None:
            # First movement, no need to rotate
            self.current_axis = new_axis
            return

        if self.current_axis != new_axis:
            # Rotate 90 degrees to align with the new axis
            if new_axis == 'x':
                print("Rotating to align with x-axis.")
                self.thymio.set_speed(self.speed, -self.speed)  # Counterclockwise rotation
            elif new_axis == 'y':
                print("Rotating to align with y-axis.")
                self.thymio.set_speed(-self.speed, self.speed)  # Clockwise rotation
            time.sleep(0.5)  # Wait for rotation to complete
            self.thymio.stop()
            self.current_axis = new_axis  # Update the current axis

    def move_to_next_target(self):
        """
        Move the robot towards the next target in the path, applying rotations as needed.
        """
        current_target = self.path[self.current_target_index]
        distance_to_target = self.compute_distance(self.current_position, current_target)

        if distance_to_target < self.lookahead_distance:
            # Arrived at the current target
            print(f"Arrived at target {current_target}.")
            self.thymio.stop()
            self.current_target_index += 1

            if self.current_target_index >= len(self.path):
                print("Path completed.")
                return  # Path completed, stop further movement

            # Determine new axis and rotate if needed
            next_target = self.path[self.current_target_index]
            new_axis = self.determine_axis(current_target, next_target)
            self.rotate_to_new_axis(new_axis)
        else:
            # Keep moving straight towards the target
            print(f"Moving towards target {current_target}")
            self.thymio.set_speed(self.speed, self.speed)

        # Update current position
        self.current_position = current_target

    def live_dynamic_local_adjustments(self):
        """
        Placeholder for dynamic local adjustments, e.g., obstacle avoidance.

        This function should be implemented later to handle dynamic changes in the environment.
        """
        pass

    def run(self):
        """
        Main control loop for motion control.

        Continuously follow the path and handle dynamic adjustments.
        """
        while self.current_target_index < len(self.path):
            self.move_to_next_target()
            self.live_dynamic_local_adjustments()  # Integrate dynamic adjustments
            time.sleep(0.1)  # Loop delay for smoother control








if __name__ == "__main__":
    thymio = ThymioController()
    #path = [(23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]
    path = [(26, 1), (25, 1), (24, 1), (23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]


    try:
        thymio.connect(timeout=5)
        motion_control = MotionControl(thymio, path)
        motion_control.run()
    finally:
        thymio.disconnect()


