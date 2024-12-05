import time
from src.motion_control.ThymioController import ThymioController
from src.Constants import BASE_LOOP_DELAY, PUT_BACK_DELAY


class LocalNavigator:
    def __init__(self, thymio):
        """
        Initialize the local navigation controller.

        Args:
            thymio (ThymioController): An instance of ThymioController.
        """
        self.thymio = thymio
        self.speed0 = 100  # Nominal speed
        self.obst_thr_high = 2000  # Proximity threshold for detecting obstacles
        self.obst_thr_low = 1500  # Proximity threshold for clearing obstacles
        self.obst_speed_gain = 0.07  # Gain for adjusting speed during obstacle avoidance
        self.ground_thr_limit = 0  # Threshold for detecting whether the robot has been kidnapped

    def kidnapping_detect(self):
        """
        Detects whether the robot has been lifted off its operating surface (kidnapping).

        Returns:
            bool: True if the robot has been kidnapped (detected via ground proximity sensors),
                  False otherwise.
        """
        prox = self.thymio.get_ground_proximity()
        return any(sensor <= self.ground_thr_limit for sensor in prox)

    def put_back_detecting(self):
        """
        Detects when the robot has been placed back on a surface after being kidnapped.

        Continuously checks the ground proximity sensors and waits until the robot is placed on a
        solid surface. Once detected, the function waits for a predefined delay before returning.
        """
        while True:
            prox = self.thymio.get_ground_proximity()
            if any(sensor > self.ground_thr_limit for sensor in prox):
                time.sleep(PUT_BACK_DELAY)
                return
            time.sleep(BASE_LOOP_DELAY)

    def obstacle_detect(self):
        """
        Check if an obstacle is detected using the front proximity sensors.

        Returns:
            bool: True if any of the front or side sensors detect an obstacle (value exceeds
                  `obst_thr_high`), False otherwise.
        """
        prox = self.thymio.get_front_proximity()
        return any(sensor > self.obst_thr_high for sensor in prox[:5])  # Consider front and side sensors

    def handle_obstacle(self):
        """
        Handle obstacle avoidance using all proximity sensor data.

        This function detects obstacles using the front proximity sensors and adjusts the robot's
        wheel speeds accordingly to avoid collisions. The robot adjusts its motor speeds based on
        the proximity readings of the sensors, steering away from the obstacle.

        """
        print("Obstacle detected! Avoiding...")
        while True:
            prox = self.thymio.get_front_proximity()

            front_left = prox[2]
            front_right = prox[3]
            side_left = prox[0]
            side_right = prox[4]

            if max(prox[:5]) > self.obst_thr_low:  # Obstacle still detected
                motor_left = self.speed0 - self.obst_speed_gain * (front_right + side_right)
                motor_right = self.speed0 - self.obst_speed_gain * (front_left + side_left)
                motor_left = max(min(motor_left, 300), -300)  # Clamp speed
                motor_right = max(min(motor_right, 300), -300)  # Clamp speed

                print(f"Avoiding: Left Speed = {motor_left}, Right Speed = {motor_right}")
                self.thymio.set_speed(int(motor_left), int(motor_right))
            else:
                print("Obstacle cleared.")
                self.thymio.stop()
                break

            time.sleep(BASE_LOOP_DELAY)
