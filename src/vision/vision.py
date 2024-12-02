import cv2
import numpy as np
import os
import yaml

class ComputerVision:
    def __init__(self, calibration_file='src/vision/calibration.yaml'):
        """
        Initializes the ComputerVision object, loads calibration data, and sets up camera capture.

        :param calibration_file: Path to the calibration file (default is 'calibration.yaml')
        """
        # Load calibration data
        with open(calibration_file, 'r') as f:
            calibration_data = yaml.safe_load(f)
        self.mtx = np.array(calibration_data['camera_matrix'])
        self.dist = np.array(calibration_data['dist_coeff'])

        # Define ARUCO dictionary to detect
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        # Initialize the video capture (0 for default camera)
        self.cap = cv2.VideoCapture(0)

        # Get the optimal camera matrix
        ret, img = self.cap.read()
        if not ret:
            print("Error: Could not read from the camera.")
            exit()
        h, w = img.shape[:2]
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))

    def get_frame(self):
        ret, img = self.cap.read()
        if not ret:
            print("Error: Could not read from the camera.")
        return ret, img

    def process_frame(self):
        """
        Processes each frame to detect ArUco markers, calculate perspective transformation, and update the grid map.
        """
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return None

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        start_point = None
        end_point = None

        if ids is not None:
            # Detect start (6) and end (5) markers
            for i, marker_id in enumerate(ids.flatten()):
                marker_center = np.mean(corners[i][0], axis=0)  # Center of the marker

                if marker_id == 6:  # START
                    start_point = marker_center
                    cv2.circle(frame, tuple(start_point.astype(int)), 5, (0, 255, 0), -1)  # Draw green circle

                elif marker_id == 5:  # END
                    end_point = marker_center
                    cv2.circle(frame, tuple(end_point.astype(int)), 5, (0, 0, 255), -1)  # Draw red circle

            # Check if all four required corner markers are detected
            required_ids = {1, 2, 3, 4}
            marker_positions = {marker_id: corners[i][0] for i, marker_id in enumerate(ids.flatten())}
            if required_ids.issubset(marker_positions.keys()):
                rect_corners = np.array([
                    marker_positions[1][0],  # Top-left corner
                    marker_positions[2][1],  # Top-right corner
                    marker_positions[3][2],  # Bottom-right corner
                    marker_positions[4][3],  # Bottom-left corner
                ], dtype=np.float32)

                # Ensure rect_corners has the correct shape
                if rect_corners.shape == (4, 2):
                    # Define destination points for the flattened rectangle (6:8 aspect ratio)
                    width, height = 600, 800  # Desired output size with 6:8 aspect ratio
                    dst_corners = np.array([
                        [0, 0],
                        [width - 1, 0],
                        [width - 1, height - 1],
                        [0, height - 1]
                    ], dtype=np.float32)

                    # Compute the perspective transform matrix
                    matrix = cv2.getPerspectiveTransform(rect_corners, dst_corners)

                    # Apply perspective warp to get the top-down view
                    warped_image = cv2.warpPerspective(frame, matrix, (width, height))

                    # Convert warped image to grayscale and threshold it to create a binary map
                    gray_warped = cv2.cvtColor(warped_image, cv2.COLOR_BGR2GRAY)
                    _, binary_map = cv2.threshold(gray_warped, 100, 255, cv2.THRESH_BINARY_INV)

                    # Mask ArUco markers 5 and 6
                    for i, marker_id in enumerate(ids.flatten()):
                        if marker_id in {5, 6}:  # Mask START (6) and END (5)
                            marker_center = np.mean(corners[i][0], axis=0)  # Center of the marker
                            warped_center = cv2.perspectiveTransform(
                                np.array([[marker_center]], dtype=np.float32), matrix)[0][0]
                            warped_center_int = tuple(np.clip(warped_center.astype(int),
                                                              0, [binary_map.shape[1] - 1, binary_map.shape[0] - 1]))
                            cv2.circle(binary_map, warped_center_int, 100, 0, -1)  # Mask as white

                    # Create a grid map for pathfinding
                    grid_map = np.zeros_like(binary_map, dtype=int)

                    # Set obstacle regions (binary_map == 255)
                    grid_map[binary_map == 255] = 1

                    # Set start and end points in the grid map
                    if start_point is not None:
                        start_warped = cv2.perspectiveTransform(np.array([[start_point]], dtype=np.float32), matrix)[0][0]
                        start_coords = tuple(start_warped.astype(int))
                        grid_map[start_coords[1], start_coords[0]] = 3  # Start

                    if end_point is not None:
                        end_warped = cv2.perspectiveTransform(np.array([[end_point]], dtype=np.float32), matrix)[0][0]
                        end_coords = tuple(end_warped.astype(int))
                        grid_map[end_coords[1], end_coords[0]] = 2  # End

                    return grid_map

        return None

    def get_map(self):
        """
        Captures the grid map from camera and returns it.
        """
        grid_map = None
        while True:
            grid_map = self.process_frame()
            if grid_map is not None:
                print("Grid map generated.")
                break

            # Exit on 'q'
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
        return grid_map


# # Usage
# #from Computer_Vision import *
# cv = ComputerVision()
# map = cv.get_map() #for Optimal path
