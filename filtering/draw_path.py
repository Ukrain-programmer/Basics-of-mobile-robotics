import cv2
from cv2 import aruco
import numpy as np
import yaml

# Load calibration data
with open('calibration.yaml', 'r') as f:
    calibration_data = yaml.safe_load(f)
mtx = np.array(calibration_data['camera_matrix'])
dist = np.array(calibration_data['dist_coeff'])
print(mtx)

# Load the ArUco dictionary and board
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
arucoParams = aruco.DetectorParameters_create()

# Set up the GridBoard parameters
markerLength = 3.75  # cm
markerSeparation = 0.5  # cm
board = aruco.GridBoard_create(4, 5, markerLength, markerSeparation, aruco_dict)

# Initialize camera
camera = cv2.VideoCapture(0)
if not camera.isOpened():
    print("Error: Could not open the camera.")
    exit()

# Get the optimal camera matrix
ret, img = camera.read()
if not ret:
    print("Error: Could not read from the camera.")
    exit()

h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# Initialize a list to track the centers of the markers
marker_centers = {}

# Processing loop
print("Press 'q' to exit.")
while True:
    ret, img = camera.read()
    if not ret:
        print("Error: Could not read from the camera.")
        break

    # Convert to grayscale
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Undistort the image
    dst = cv2.undistort(img_gray, mtx, dist, None, newcameramtx)
    
    # Detect markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, aruco_dict, parameters=arucoParams)
    
    if ids is not None:  # Markers detected
        # Estimate pose of the board
        rvec = np.zeros((1, 3), dtype=np.float32)
        tvec = np.zeros((1, 3), dtype=np.float32)
        ret = aruco.estimatePoseBoard(corners, ids, board, newcameramtx, dist, rvec, tvec)
        
        if ret:
            # Draw markers and axis
            img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0, 255, 0))
            img_aruco = cv2.drawFrameAxes(img_aruco, newcameramtx, dist, rvec, tvec, 20)

            # Get the 3D positions of the marker corners in the world frame
            marker_corners_3d = np.array([
                [-markerLength / 2, -markerLength / 2, 0],
                [markerLength / 2, -markerLength / 2, 0],
                [markerLength / 2, markerLength / 2, 0],
                [-markerLength / 2, markerLength / 2, 0]
            ], dtype=np.float32)

            # Solve PnP for each marker
            for i in range(len(ids)):
                # Get the corresponding 2D points for this marker
                marker_2d = corners[i][0]
                
                # Solve PnP to find rotation and translation vectors for each marker
                _, rvec_marker, tvec_marker = cv2.solvePnP(marker_corners_3d, marker_2d, newcameramtx, dist)
                
                # Project the 3D points of the marker to 2D image coordinates
                projected_points, _ = cv2.projectPoints(marker_corners_3d, rvec_marker, tvec_marker, newcameramtx, dist)
                
                # Convert to integer coordinates
                projected_points = projected_points.reshape(-1, 2).astype(int)
                
                # Calculate the center of the marker
                center = np.mean(projected_points, axis=0).astype(int)

                # Store the center for later use
                marker_id = ids[i][0]
                if marker_id not in marker_centers:
                    marker_centers[marker_id] = []

                marker_centers[marker_id].append(center)

                # Draw circles on the marker center
                cv2.circle(img_aruco, tuple(center), 5, (0, 0, 255), -1)

                # Optionally, draw lines between the centers to show the movement path
                if len(marker_centers[marker_id]) > 1:
                    for j in range(1, len(marker_centers[marker_id])):
                        cv2.line(img_aruco, tuple(marker_centers[marker_id][j-1]), tuple(marker_centers[marker_id][j]), (255, 0, 0), 2)

            # Display the output
            cv2.imshow("World Coordinate Frame Axes", img_aruco)
        else:
            print("Pose estimation failed.")
    else:
        print("No markers detected.")

    # Quit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
camera.release()
cv2.destroyAllWindows()
