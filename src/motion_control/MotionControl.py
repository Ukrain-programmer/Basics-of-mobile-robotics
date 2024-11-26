import time

from ThymioController import ThymioController
from LocalNavigator import LocalNavigator

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
        self.current_position = global_path[0]
        self.setup_orientation()  # Start facing +X (0 degrees)
        self.local_navigator = LocalNavigator(thymio_controller)
        self.duration = 0.5
        self.speed = 150

    def setup_orientation(self):
        if len(self.path) > 1:  # Ensure there are at least two waypoints
            x1, y1 = self.path[0]
            x2, y2 = self.path[1]

            if x2 > x1:
                self.current_orientation = 0  # Facing +X
            elif x2 < x1:
                self.current_orientation = 180  # Facing -X
            elif y2 > y1:
                self.current_orientation = 90  # Facing +Y
            elif y2 < y1:
                self.current_orientation = 270  # Facing -Y
        else:
            self.current_orientation = 0
    def calculate_target_orientation(self, target_position):
        """
        Calculate the target orientation based on the next waypoint.
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
        """
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

    def move_to_target(self, target_position):
        """
        Move the robot to the target position, avoiding obstacles if necessary.

        Args:
            target_position (tuple): The target (x, y) position.
        """
        print(f"Moving to target: {target_position}")
        end_time = time.time() + self.duration
        while time.time() < end_time:
            # Check for obstacles
            if self.local_navigator.obstacle_detected():
                self.thymio.stop()
                self.local_navigator.handle_obstacle()
            else:
                # No obstacle, move forward
                self.thymio.set_speed(self.speed, self.speed)

            time.sleep(0.1)  # Sampling delay

        # self.thymio.stop()
        self.current_position = target_position  # Update position (simulated)

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

            print(f"Reached waypoint: {target_position}")

        self.thymio.stop()
        print("Path execution complete.")





if __name__ == "__main__":
    thymio = ThymioController()
    path = [(29, 16), (29, 17), (29, 18), (29, 19), (29, 20), (29, 21), (29, 22), (29, 23), (30, 23), (31, 23), (32, 23), (33, 23), (34, 23), (35, 23), (36, 23), (36, 24), (37, 24), (38, 24), (38, 25), (39, 25), (39, 26), (39, 27), (40, 27), (40, 28), (40, 29), (40, 30), (40, 31), (40, 32), (40, 33), (40, 34), (40, 35), (40, 36), (41, 36), (42, 36), (43, 36), (44, 36), (45, 36), (46, 36), (47, 36), (48, 36), (49, 36), (50, 36), (51, 36), (52, 36), (53, 36), (54, 36), (55, 36), (56, 36), (57, 36), (58, 36), (59, 36), (60, 36), (61, 36), (62, 36), (63, 36), (64, 36), (65, 36), (66, 36), (67, 36), (68, 36), (69, 36)]
    try:
        # Define path (list of waypoints)
        #path = [(19, 1), (20, 1), (21, 1), (21, 2), (21, 3), (21, 4)]
        thymio.connect(timeout=5)
        motion_control = MotionControl(thymio, path)
        motion_control.execute_path()

    finally:
        thymio.disconnect()