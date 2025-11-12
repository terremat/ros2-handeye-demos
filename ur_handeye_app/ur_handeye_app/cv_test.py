import cv2
import numpy as np

# Load the image
image = cv2.imread('experiments/image.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Checkerboard dimensions (number of inner corners per a chessboard row and column)
# Note: If your board is 8x9 squares, inner corners are 7x8
checkerboard_size = (8, 7)  # (columns, rows) of inner corners

cv2.imshow('Corners', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Find the checkerboard corners
ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

if ret:
    # Refine corner locations to subpixel accuracy
    criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
    corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

    # Draw and display the corners
    cv2.drawChessboardCorners(image, checkerboard_size, corners_refined, ret)
    cv2.imshow('Corners', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Optionally, print corner coordinates
    print("Detected corners:")
    print(corners_refined)
else:
    print("Checkerboard corners not found.")
