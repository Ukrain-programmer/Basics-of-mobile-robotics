import time
from tdmclient import ClientAsync, aw


class ThymioController:
    def __init__(self):
        """
        Initialize the ThymioController with a synchronous client instance.
        """
        self.client = ClientAsync()
        self.node = None

    def connect(self, timeout=5):
        """
        Connect to the Thymio robot by checking for available nodes, with a timeout.

        Args:
            timeout (int): Maximum time to wait for a connection (in seconds).
        """
        try:
            print("Searching for Thymio...")
            start_time = time.time()

            while time.time() - start_time < timeout:
                self.client.process_waiting_messages()  # Update client state
                for node in self.client.nodes:
                    self.node = node
                    break
                if self.node:
                    aw(self.node.lock())
                    print("Thymio connected!")
                    return
                time.sleep(0.1)

            if not self.node:
                print("No Thymio robot found within the timeout period.")
        except Exception as e:
            print(f"Failed to connect to Thymio: {e}")

    def set_speed(self, left_speed, right_speed):
        """
        Set the motor speeds of the Thymio robot.

        Args:
            left_speed (int): Speed for the left motor.
            right_speed (int): Speed for the right motor.
        """
        if self.node:
            v = {
                "motor.left.target": [left_speed],
                "motor.right.target": [right_speed],
            }
            aw(self.node.set_variables(v))
        else:
            print("Thymio not connected. Please connect first.")

    def stop(self):
        """
        Stop the Thymio robot by setting both motor speeds to zero.
        """
        if self.node:
            self.set_speed(0, 0)
            print("Thymio stopped.")
        else:
            print("Thymio not connected. Please connect first.")

    def disconnect(self):
        """
        Disconnect from the Thymio robot and close the client connection.
        """
        if self.node:
            self.stop()
            aw(self.node.unlock())

        if self.client:
            self.client.close()
            print("Disconnected from Thymio.")

    def get_front_proximity(self):
        """
        Retrieve the current proximity sensor values as an array.

        Returns:
            list: An array of proximity sensor values [front_left, front_left_center, front_center, front_right_center, front_right],
                  or None if the Thymio is not connected.
        """
        if self.node:
            self.client.process_waiting_messages()
            aw(self.node.wait_for_variables({"prox.horizontal"}))
            prox_horizontal = list(self.node["prox.horizontal"])

            return [
                prox_horizontal[0],
                prox_horizontal[1],
                prox_horizontal[2],
                prox_horizontal[3],
                prox_horizontal[4],
            ]
        else:
            print("Thymio not connected. Please connect first.")
            return None

    def get_ground_proximity(self):
        """
        Retrieve the current ground sensor values as an array.

        Returns:
            list: An array of ground sensor values [left_ground, right_ground],
                  or None if the Thymio is not connected.
        """
        if self.node:
            self.client.process_waiting_messages()
            aw(self.node.wait_for_variables({"prox.ground.ambiant"}))
            ground_reflected = list(self.node["prox.ground.ambiant"])

            return [
                ground_reflected[0],  # Left ground sensor
                ground_reflected[1],  # Right ground sensor
            ]
        else:
            print("Thymio not connected. Please connect first.")
            return None

    def get_speed(self):
        """
        Retrieve the current motor speeds as an array.

        Returns:
            list: An array of motor speeds [left_speed, right_speed],
                  or None if the Thymio is not connected.
        """
        if self.node:
            self.client.process_waiting_messages()
            aw(self.node.wait_for_variables({"motor.left.speed", "motor.right.speed"}))

            left_speed = self.node["motor.left.speed"]
            right_speed = self.node["motor.right.speed"]

            return left_speed, right_speed
        else:
            print("Thymio not connected. Please connect first.")
            return None
