
import time
import math
from ThymioController import ThymioController

MAX_SPEED = 500  # Maximum motor speed
BASE_SPEED = 200  # Base speed for the robot
SPEED_ADJUSTMENT = 100  # Adjustment factor for speed based on distance error
HEADING_CORRECTION_FACTOR = 100  # Factor for heading correction speed adjustment
WAYPOINT_THRESHOLD = 0.05  # Threshold distance to consider a waypoint reached
TIME_STEP = 0.1  # Time step for filtering (seconds)
KALMAN_DT = 0.1  # Time step for the Kalman filter (seconds)

class MotionControl:
    def __init__(self, thymio_controller, kalman_filter, wheel_base=0.1, kp=1.0, kd=0.5):
        """
        Initialize the MotionControl class.

        Args:
            thymio_controller (ThymioController): Instance of ThymioController to control the robot.
            kalman_filter (function): Kalman filtering function to estimate position and heading.
            wheel_base (float): Distance between the wheels (in meters).
            kp (float): Proportional gain for the PD controller.
            kd (float): Derivative gain for the PD controller.
        """
        self.thymio = thymio_controller
        self.kalman_filter = kalman_filter
        self.wheel_base = wheel_base
        self.kp = kp
        self.kd = kd
        self.previous_error = 0
        self.previous_time = None
        self.current_state = [0, 0, 0]  # Initialize state [x, y, theta]

    def follow_path(self, path):
        """
        Follow a given path smoothly.

        Args:
            path (list of tuples): A list of (x, y) waypoints defining the path.
        """
        if not path:
            print("Path is empty, stopping robot.")
            self.thymio.stop()
            return

        current_index = 0  # Start at the first waypoint

        while current_index < len(path):
            # Get the left and right wheel velocities
            left_velocity, right_velocity = self._get_wheel_velocities()

            # Update the current state using the Kalman filter
            self.current_state = self.kalman_filter(self.current_state, left_velocity, right_velocity, self.wheel_base)
            current_pos = self.current_state[:2]  # Current position (x, y)
            current_theta = self.current_state[2]  # Current heading

            # Get the target waypoint
            target_pos = path[current_index]

            # Compute control errors
            dx = target_pos[0] - current_pos[0]
            dy = target_pos[1] - current_pos[1]
            distance_error = math.sqrt(dx**2 + dy**2)
            target_theta = math.atan2(dy, dx)
            heading_error = self._normalize_angle(target_theta - current_theta)

            # Check if the waypoint is reached
            if distance_error < WAYPOINT_THRESHOLD:  # Threshold for reaching the waypoint
                current_index += 1
                continue

            # Compute control signals (PD Controller)
            delta_time = self._delta_time()
            heading_correction = self.kp * heading_error + self.kd * (heading_error - self.previous_error) / delta_time
            self.previous_error = heading_error

            # Convert control signals to motor speeds
            left_speed, right_speed = self._compute_motor_speeds(distance_error, heading_correction)
            self.thymio.set_speed(left_speed, right_speed)

            # Simulate time delay
            time.sleep(TIME_STEP)

        self.thymio.stop()

    def _compute_motor_speeds(self, distance_error, heading_correction):
        """
        Compute motor speeds based on distance error and heading correction.

        Args:
            distance_error (float): The distance to the target.
            heading_correction (float): The correction to align the heading.

        Returns:
            tuple: Speeds for the left and right motors.
        """
        base_speed = min(MAX_SPEED, BASE_SPEED + distance_error * SPEED_ADJUSTMENT)  # Adjust speed dynamically
        left_speed = int(base_speed - heading_correction * HEADING_CORRECTION_FACTOR)
        right_speed = int(base_speed + heading_correction * HEADING_CORRECTION_FACTOR)
        return max(-MAX_SPEED, min(MAX_SPEED, left_speed)), max(-MAX_SPEED, min(MAX_SPEED, right_speed))

    def _normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].

        Args:
            angle (float): Angle in radians.

        Returns:
            float: Normalized angle.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _delta_time(self):
        """
        Compute the time delta since the last call.

        Returns:
            float: Time delta in seconds.
        """
        current_time = time.time()
        if self.previous_time is None:
            self.previous_time = current_time
            return TIME_STEP  # Default time step
        delta = current_time - self.previous_time
        self.previous_time = current_time
        return delta

    def _get_wheel_velocities(self):
        """
        Get the current velocities of the left and right wheels.

        Returns:
            tuple: Velocities of the left and right wheels (in m/s).
        """
        # Mock implementation: Replace with actual velocity measurement
        left_velocity = 0.1  # Example value in m/s
        right_velocity = 0.1  # Example value in m/s
        return left_velocity, right_velocity

    def avoid_local_obstacle(self):
        """
        Local navigation for avoiding obstacles.
        To be implemented.
        """
        pass




if __name__ == "__main__":
    thymio = ThymioController()
    try:

        thymio.connect(timeout=5)

        # Initialize MotionControl with ThymioController and Kalman filter
        motion_control = MotionControl(thymio)

        # Example path (list of waypoints)
        path = [(29, 16), (29, 17), (29, 18), (29, 19)]

        # Follow the path
        motion_control.follow_path(path)

    finally:
        thymio.disconnect()



#%%
