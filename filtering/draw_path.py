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

# Load the ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
arucoParams = aruco.DetectorParameters_create()

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

# Store the positions of the circles to create a path effect
circle_positions = []

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
        # Draw detected markers
        img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0, 255, 0))
        
        # Estimate pose for each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 3.75, newcameramtx, dist)
        
        # Process each detected marker
        for i in range(len(ids)):
            # Draw axis for each marker
            img_aruco = cv2.drawFrameAxes(img_aruco, newcameramtx, dist, rvecs[i], tvecs[i], 20)
            
            # Get the corner coordinates of the current marker
            marker_corners = corners[i][0]
            
            # Compute the center of the marker (average of the 4 corners)
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            
            # Draw a circle at the center of the marker
            radius = 10  # radius of the circle in pixels
            cv2.circle(img_aruco, (center_x, center_y), radius, (0, 0, 255), -1)  # Red circle
            
            # Store the center for path drawing
            circle_positions.append((center_x, center_y))
        
        # Draw the path (previous circles) by connecting the circle centers
        for i in range(1, len(circle_positions)):
            cv2.line(img_aruco, circle_positions[i - 1], circle_positions[i], (255, 0, 0), 2)  # Blue path

        # Display the output
        cv2.imshow("World Coordinate Frame Axes with Path", img_aruco)
    else:
        print("No markers detected.")

    # Quit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
camera.release()
cv2.destroyAllWindows()

