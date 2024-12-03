import numpy as np
import cv2
import yaml


class Image_EKF:
    def __init__(self):
        pass

    @staticmethod
    def initialize_camera_and_calibration(calibration_file='src/vision/calibration.yaml', camera_index=1,
                                          aruco_dict_type=cv2.aruco.DICT_6X6_1000):

        with open(calibration_file, 'r') as f:
            calibration_data = yaml.safe_load(f)

        mtx = np.array(calibration_data['camera_matrix'])
        dist = np.array(calibration_data['dist_coeff'])

        # Load the ArUco dictionary and parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        aruco_params = cv2.aruco.DetectorParameters()

        # Initialize the camera
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            raise RuntimeError("Error: Could not open the camera.")

        # Read a frame to determine the image size
        ret, img = camera.read()
        if not ret:
            raise RuntimeError("Error: Could not read from the camera.")

        h, w = img.shape[:2]

        # Get the optimal camera matrix
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        return camera, aruco_dict, aruco_params, newcameramtx, dist, mtx

    @staticmethod
    def detect_marker_with_id(frame, aruco_dict, aruco_params, target_id=10):
        robot_detected = False
        # Detect markers in the frame
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == target_id:
                    return corners[i], marker_id, True

        # Return None if the target ID is not found
        return None, None, False

    @staticmethod
    def process_marker(corners, marker_id, target_id, camera_matrix, dist_coeffs, marker_centers):

        if marker_id != target_id:
            return None, None, None, None

        # Get the 2D positions of the marker corners
        marker_2d = corners[0]

        # Define the 3D positions of the marker corners in the world frame
        marker_corners_3d = np.array([
            [-0.45, -0.45, 0],
            [0.45, -0.45, 0],
            [0.45, 0.45, 0],
            [-0.45, 0.45, 0]
        ], dtype=np.float32)

        # Solve PnP to find rotation and translation vectors
        _, rvec_marker, tvec_marker = cv2.solvePnP(marker_corners_3d, marker_2d, camera_matrix, dist_coeffs)

        # Project the 3D points to 2D
        projected_points, _ = cv2.projectPoints(marker_corners_3d, rvec_marker, tvec_marker, camera_matrix, dist_coeffs)
        projected_points = projected_points.reshape(-1, 2).astype(int)

        # Calculate the center of the marker
        center = tuple(np.mean(projected_points, axis=0).astype(int))
        marker_centers.append(center)

        return center, projected_points, tvec_marker, rvec_marker, marker_centers, marker_corners_3d

    @staticmethod
    def resize_image(image, new_width=60):

        # Get the original dimensions of the image
        height, width = image.shape[:2]

        # Calculate the new height to maintain the aspect ratio
        new_height = int((new_width / width) * height)

        # Resize the image using nearest neighbor interpolation
        resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

        return resized_image
