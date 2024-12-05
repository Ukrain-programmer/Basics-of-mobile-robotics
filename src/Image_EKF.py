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

        
        camera = cv2.VideoCapture(0)
        if not camera.isOpened():
            raise RuntimeError("Error: Could not open the camera.")

       
        ret, img = camera.read()
        if not ret:
            raise RuntimeError("Error: Could not read from the camera.")

        h, w = img.shape[:2]

        
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
        
    @staticmethod
    def warp_and_transform(frame, width=60, height=80,point=None):
        rect_corners = np.array([[137, 154],[485, 158],[517, 425],[ 98,424]], dtype=np.float32)
        # Define destination points for the flattened rectangle
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
        if point is not None:
            # Ensure point is in homogeneous coordinates
            point = np.array([point[0], point[1], 1], dtype=np.float32)  # Add z = 1 for homogeneity
            point_warped = np.dot(matrix, point)
            point_warped /= point_warped[2]  # Normalize to get 2D coordinates
            point_warped = tuple(map(int, point_warped[:2])) 
        
            return warped_image, point_warped[:2]  # Return x, y as 2D
        return warped_image 
        
        
