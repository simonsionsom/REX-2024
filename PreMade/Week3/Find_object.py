import cv2
import numpy as np
import glob
import os

# Termination criteria for the cornerSubPix function
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D points in real world space)
# In this case, we use a chessboard with 9x6 corners (this can vary based on your chessboard)
objp = np.zeros((6*9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)  # x, y coordinates of the chessboard squares

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load all calibration images (you should have multiple chessboard images from different angles)
images = glob.glob('samples/data/left01.jpg–left14.jpg)')

if not images:
    raise FileNotFoundError("No calibration images found in the specified directory.")

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Failed to load image {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

    # If found, add object points and image points (after refining them)
    if ret:
        objpoints.append(objp)

        # Refine corner locations
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9, 6), corners2, ret)
        if os.environ.get('DISPLAY'):
            cv2.imshow('Chessboard', img)
            cv2.waitKey(500)

if os.environ.get('DISPLAY'):
    cv2.destroyAllWindows()

# Calibrate the camera using the object points and image points
if objpoints and imgpoints:
    ret, intrinsic_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Print the results
    print("Camera intrinsic matrix:\n", intrinsic_matrix)
    print("Distortion coefficients:\n", distortion_coeffs)
else:
    print("Calibration failed. No chessboard corners found in any image.")

