from tdmclient import ClientAsync, aw
import time
import math
import asyncio

class ThymioController:
    def __init__(self):
        """
        Initialize the ThymioController with a synchronous client instance.
        """
        self.client = ClientAsync()
        self.node = None
        self.wheel_speed = [0, 0] #current wheel speed (left, right)
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
            left_speed (int): Speed for the left motor (-500 to 500).
            right_speed (int): Speed for the right motor (-500 to 500).
        """
        if self.node:
            v = {
                "motor.left.target": [left_speed],
                "motor.right.target": [right_speed],
            }
            aw(self.node.set_variables(v))
            self.wheel_speed[0] = left_speed
            self.wheel_speed[1] = right_speed
            # print(f"Set motor speeds: left={left_speed}, right={right_speed}")
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



    def turn_right(self, speed = 200):
        self.set_speed(speed, -speed)
        time.sleep(1.1)
        self.stop()

    def turn_left(self, speed = 200):
        self.set_speed(-speed, speed)
        time.sleep(1.1)
        self.stop()

    def get_front_proximity(self):
        """
        Retrieve the current proximity sensor values as an array.

        Returns:
            list: An array of proximity sensor values [front_left, front_left_center, front_center, front_right_center, front_right],
                  or None if the Thymio is not connected.
        """
        global leds_top, leds_bottom_left
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







if __name__ == "__main__":
    try:
        thymio = ThymioController()
        thymio.connect(timeout=5)

        v = thymio.get_ground()
        thymio.turn_right(200)

    finally:
        thymio.disconnect()
