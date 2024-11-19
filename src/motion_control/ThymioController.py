from tdmclient import ClientAsync, aw
import time


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
            left_speed (int): Speed for the left motor (-500 to 500).
            right_speed (int): Speed for the right motor (-500 to 500).
        """
        if self.node:
            v = {
                "motor.left.target": [left_speed],
                "motor.right.target": [right_speed],
            }
            aw(self.node.set_variables(v))
            print(f"Set motor speeds: left={left_speed}, right={right_speed}")
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


# Usage example
# if __name__ == "__main__":
#     thymio = ThymioController()
#     try:
#         thymio.connect(timeout=5)
#         thymio.set_speed(0, 0)
#         # time.sleep(5)
#         # thymio.stop()
#     finally:
#         thymio.disconnect()
