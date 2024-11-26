import time
from ThymioController import ThymioController

class LocalNavigator:
    def __init__(self, thymio):
        """
        Initialize the local navigation controller.

        Args:
            thymio (ThymioController): An instance of ThymioController.
        """
        self.thymio = thymio
        self.state = 0  # 0 = goal tracking, 1 = obstacle avoidance
        self.speed0 = 100  # Nominal speed
        self.speed_gain = 2  # Gain for gradient tracking
        self.obst_thr_high = 2000  # Proximity threshold to start avoidance
        self.obst_thr_low = 1500  # Proximity threshold to return to goal tracking
        self.obst_speed_gain = 0.05  # Gain for obstacle avoidance (scaled)

    def obstacle_detected(self):
        """
        Check if an obstacle is detected based on proximity sensor values.

        Returns:
            bool: True if an obstacle is detected, False otherwise.
        """
        prox = self.thymio.get_proximity()
        left_obst = prox[0]
        right_obst = prox[4]
        return left_obst > self.obst_thr_high or right_obst > self.obst_thr_high

    def handle_obstacle(self):
        """
        Handle obstacle avoidance by switching to obstacle avoidance mode.
        """
        print("Handling obstacle...")
        while True:
            prox = self.thymio.get_proximity()
            left_obst = prox[0]
            right_obst = prox[4]

            # Obstacle detected, perform avoidance
            if left_obst > self.obst_thr_high or right_obst > self.obst_thr_high:
                motor_left = int(self.speed0 + self.obst_speed_gain * left_obst)
                motor_right = int(self.speed0 + self.obst_speed_gain * right_obst)
                print(f"Avoiding: Left Speed = {motor_left}, Right Speed = {motor_right}")
                self.thymio.set_speed(motor_left, motor_right)

            else:
                # Obstacle cleared
                print("Obstacle cleared.")
                self.thymio.stop()
                break

            time.sleep(0.1)  # Sampling delay


# class LocalNavigator:
#     def __init__(self, thymio):
#         """
#         Initialize the local navigation controller.
#
#         Args:
#             thymio (ThymioController): An instance of ThymioController.
#         """
#         self.thymio = thymio
#         self.state = 0  # 0 = goal tracking, 1 = obstacle avoidance
#         self.speed0 = 100  # Nominal speed
#         self.speed_gain = 2  # Gain for gradient tracking
#         self.obst_thr_high = 2000  # Proximity threshold to start avoidance
#         self.obst_thr_low = 1500  # Proximity threshold to return to goal tracking
#         self.obst_speed_gain = 0.05  # Gain for obstacle avoidance (scaled)
#
#     def goal_tracking(self):
#         """
#         Navigate toward the goal using ground sensors to measure the gradient.
#         """
#         diff_delta = self.thymio.get_ground_delta()  # Difference in ground sensor readings
#         motor_left = self.speed0 - self.speed_gain * diff_delta
#         motor_right = self.speed0 + self.speed_gain * diff_delta
#         self.thymio.set_speed(motor_left, motor_right)
#         print(f"Goal Tracking: Left Speed = {motor_left}, Right Speed = {motor_right}")
#
#     def obstacle_avoidance(self):
#         """
#         Avoid obstacles by accelerating the wheel on the side of the detected obstacle.
#         """
#         prox = self.thymio.get_proximity()  # Proximity sensor values
#         left_obst = prox[0]
#         right_obst = prox[4]
#
#         motor_left = self.speed0 + self.obst_speed_gain * left_obst
#         motor_right = self.speed0 + self.obst_speed_gain * right_obst
#         self.thymio.set_speed(motor_left, motor_right)
#         print(f"Obstacle Avoidance: Left Speed = {motor_left}, Right Speed = {motor_right}")
#
#     def navigate_to_waypoint(self, duration):
#         """
#         Navigate to a waypoint, alternating between goal tracking and obstacle avoidance.
#
#         Args:
#             duration (float): Duration to navigate in seconds.
#         """
#         end_time = time.time() + duration
#         while time.time() < end_time:
#             prox = self.thymio.get_proximity()
#             left_obst = prox[0]
#             right_obst = prox[4]
#
#             if self.state == 0 and (left_obst > self.obst_thr_high or right_obst > self.obst_thr_high):
#                 # Switch to obstacle avoidance
#                 print("Obstacle detected. Switching to avoidance mode.")
#                 self.state = 1
#             elif self.state == 1 and left_obst < self.obst_thr_low and right_obst < self.obst_thr_low:
#                 # Return to goal tracking
#                 print("Obstacle cleared. Returning to goal tracking.")
#                 self.state = 0
#
#             if self.state == 0:
#                 self.goal_tracking()
#             else:
#                 self.obstacle_avoidance()
#
#             time.sleep(0.1)  # 10ms sampling time