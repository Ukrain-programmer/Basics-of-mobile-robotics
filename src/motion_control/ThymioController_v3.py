
import time
import math
from ThymioController import ThymioController
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
        self.current_orientation = 0  # Start facing +X (0 degrees)
        self.duration = 0.5

    def calculate_target_orientation(self, target_position):
        """
        Calculate the target orientation based on the next waypoint.

        Args:
            target_position (tuple): The (x, y) coordinates of the target waypoint.

        Returns:
            int: Target orientation in degrees (0, 90, 180, 270).
        """
        x1, y1 = self.current_position
        x2, y2 = target_position

        if x2 > x1:  # Moving right (+X)
            return 0
        elif x2 < x1:  # Moving left (-X)
            return 180
        elif y2 > y1:  # Moving up (+Y)
            return 90
        elif y2 < y1:  # Moving down (-Y)
            return 270

    def align_to_target_orientation(self, target_orientation):
        """
        Align the robot's orientation to the target orientation using 90-degree turns.

        Args:
            target_orientation (int): The target orientation in degrees (0, 90, 180, 270).
        """
        angle_difference = (target_orientation - self.current_orientation) % 360

        if angle_difference == 90:  # Turn right
            self.thymio.turn_right()
            self.current_orientation = (self.current_orientation + 90) % 360
        elif angle_difference == 270:  # Turn left (shortest way to rotate counterclockwise)
            self.thymio.turn_left()
            self.current_orientation = (self.current_orientation - 90) % 360
        elif angle_difference == 180:
            print("Unexpected 180-degree rotation required. Manual intervention needed.")
        else:
            print("Already facing the correct direction.")

    def move_to_target(self, target_position):
        """
        Move the robot to the target position.

        Args:
            target_position (tuple): The target (x, y) position.
        """
        print(f"Moving to target: {target_position}")
        end_time = time.time() + self.duration
        self.thymio.set_speed(200, 200)

        while time.time() < end_time:
            time.sleep(0.05)

        # Simulate movement (update current position)
        self.current_position = target_position

    def avoid_obstacle(self):
        """
        Avoid an obstacle detected using proximity sensors and adjust the robot's path.
        """
        print("Obstacle detected! Initiating avoidance maneuver.")
        while True:
            # Fetch proximity sensor values
            proximity = self.thymio.get_proximity()
            left, front_left, front_center, front_right, right = proximity

            if front_center > 2000 or front_left > 2000 or front_right > 2000:  # Obstacle ahead
                if left < right:  # More space on the left
                    print("Turning left to avoid obstacle.")
                    self.thymio.turn_left()
                    self.thymio.set_speed(200, 200)
                    time.sleep(0.5)
                    self.thymio.turn_right()
                else:  # More space on the right
                    print("Turning right to avoid obstacle.")
                    self.thymio.turn_right()
                    self.thymio.set_speed(200, 200)
                    time.sleep(0.5)
                    self.thymio.turn_left()
            elif left > 2000:
                print("Avoiding obstacle on the left.")
                self.thymio.set_speed(200, 200)
                time.sleep(0.5)
                self.current_position = (
                    self.current_position[0],
                    self.current_position[1] + 1
                )
            elif right > 2000:
                print("Avoiding obstacle on the right.")
                self.thymio.set_speed(200, 200)
                time.sleep(0.5)
                self.current_position = (
                    self.current_position[0],
                    self.current_position[1] + 1
                )
            else:
                print("Obstacle cleared. Returning to the path.")
                self.thymio.stop()
                break

    def execute_path_with_obstacle_avoidance(self):
        """
        Execute the path while dynamically avoiding obstacles.
        """
        for i in range(len(self.path) - 1):
            target_position = self.path[i + 1]

            while True:
                # Fetch proximity sensor values
                proximity = self.thymio.get_proximity()
                if any(sensor > 2000 for sensor in proximity):  # Obstacle detected
                    self.avoid_obstacle()

                    # Recalculate orientation toward the next waypoint
                    target_orientation = self.calculate_target_orientation(target_position)
                    self.align_to_target_orientation(target_orientation)

                    # Check if the robot is close enough to the target
                    if math.dist(self.current_position, target_position) < 1.0:
                        print(f"Skipped to waypoint: {target_position}")
                        break
                else:
                    break

            # Calculate target orientation
            target_orientation = self.calculate_target_orientation(target_position)

            # Align to target orientation
            self.align_to_target_orientation(target_orientation)

            # Move to the target position
            self.move_to_target(target_position)

            print(f"Reached waypoint: {target_position}")

        self.thymio.stop()
        print("Path execution complete.")


if __name__ == "__main__":
    thymio = ThymioController()
    path = [
        (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4),
        (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10)
    ]

    try:
        thymio.connect(timeout=5)
        motion_control = MotionControl(thymio, path)
        motion_control.execute_path_with_obstacle_avoidance()
    finally:
        thymio.disconnect()



if __name__ == "__main__":
    thymio = ThymioController()

    try:
        thymio.connect(timeout=5)

        path = [(22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]

        motion_control = MotionControl(thymio, path)
        motion_control.execute_path_with_obstacle_avoidance()

    finally:
        thymio.disconnect()

#%%
