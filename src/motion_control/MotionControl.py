import math

from ThymioController import ThymioController


class MotionControl:
    def __init__(self, thymio_controller: ThymioController):
        """
        MotionControl class to handle movement control and path following.
        Args:
            thymio_controller (ThymioController): Instance of ThymioController to control the robot.
        """
        self.thymio = thymio_controller

    def move_towards_goal(self, goal_x, goal_y, current_x, current_y, current_theta, angle_tolerance=4, speed=200):
        """
        Move the Thymio robot towards a goal position while following a straight path.
        Arguments:
            goal_x (float): X coordinate of the goal.
            goal_y (float): Y coordinate of the goal.
            current_x (float): Current X coordinate of the robot.
            current_y (float): Current Y coordinate of the robot.
            current_theta (float): Current orientation of the robot (in radians).
            angle_tolerance (float): Tolerance for the angle deviation (in radians).
            speed (int): Base speed for the robot.
        """
        # Calculate the desired angle to the goal
        desired_angle = math.atan2(goal_y - current_y, goal_x - current_x)

        # Calculate the angle difference (error)
        angle_error = desired_angle - current_theta

        # Normalize the angle to be within -pi to pi
        angle_error = self.wrap_angle(angle_error)

        # Check if the robot is close enough to the target angle and should just go straight
        if abs(angle_error) < angle_tolerance:
            left_speed = right_speed = speed  # Move straight
        else:
            # If the angle error is large, adjust the robot's speed to rotate
            left_speed = int(speed - angle_error * 200)  # Increase the factor to make turning faster
            right_speed = int(speed + angle_error * 200)

        # Set the robot's speed
        self.thymio.set_speed(left_speed, right_speed)

        # Return the current angle error for possible debugging or logging
        return angle_error

    def wrap_angle(self, angle):
        """
        Normalize the angle to be between -pi and pi.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def follow_path(self, path, initial_position):
        """
        Follow the provided path towards the goal. The path is a list of (x, y) points.
        Arguments:
            path (list of tuples): List of (x, y) coordinates representing waypoints.
            initial_position (tuple): The initial position (x, y, theta) of the robot.
        """
        current_x, current_y, current_theta = initial_position
        for goal_x, goal_y in path:
            print(f"Moving towards goal: ({goal_x}, {goal_y})")

            # Continue moving towards the goal until we are close enough
            while True:
                # Move the robot towards the goal
                angle_error = self.move_towards_goal(goal_x, goal_y, current_x, current_y, current_theta)

                # Update the current position (assumed to be provided by filtering/localization in the future)
                current_x, current_y = goal_x, goal_y  # Update based on the filter (e.g., localization)
                current_theta += angle_error  # This would also be updated by the filter

                # Normalize current_theta to stay within the range [-pi, pi]
                current_theta = self.wrap_angle(current_theta)

                # If we're close enough to the goal, break out of the loop
                if abs(angle_error) < 0.1 and self.distance(current_x, current_y, goal_x, goal_y) < 50:
                    print(f"Reached goal: ({goal_x}, {goal_y})")
                    break

    def distance(self, x1, y1, x2, y2):
        """
        Calculate Euclidean distance between two points.
        """
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)








if __name__ == "__main__":
    thymio = ThymioController()
    motion_control = MotionControl(thymio)
    path = [(23, 1), (22, 1), (21, 1), (20, 1), (19, 1), (19, 2), (19, 3), (19, 4), (19, 5), (19, 6), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (25, 20), (25, 21), (25, 22), (25, 23), (25, 24), (25, 25), (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (16, 27), (16, 28), (16, 29), (16, 30), (16, 31), (16, 32), (16, 33), (16, 34), (16, 35), (16, 36), (16, 37), (16, 38), (16, 39), (16, 40), (15, 40), (14, 40), (13, 40), (12, 40), (11, 40), (10, 40), (9, 40), (8, 40), (7, 40), (6, 40), (5, 40), (4, 40), (3, 40), (2, 40), (1, 40), (1, 41), (1, 42), (1, 43), (1, 44), (1, 45), (1, 46), (1, 47), (1, 48), (1, 49), (1, 50), (1, 51), (1, 52), (1, 53), (1, 54), (1, 55), (1, 56), (1, 57)]

    try:
        thymio.connect(timeout=5)
        #path = [(40, 1), (39, 1), (38, 1), (37, 1), (36, 1), (35, 1), (34, 1), (33, 1), (32, 1), (31, 1), (30, 1), (29, 1), (28, 1)]
        motion_control.follow_path(path, (23, 1, 0.2))
    finally:
        thymio.disconnect()


