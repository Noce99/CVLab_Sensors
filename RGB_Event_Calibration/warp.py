import cv2
import numpy as np

def undistort(img, K, D):
    h, w = img.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1)
    return cv2.undistort(img, K, D, None, newK)


def compute_homography(K_left, K_right, R, T):
    # Assume plane normal Z
    n = np.array([[0, 0, 1]]).T
    d = 1.0

    H = K_right @ (R - (T @ n.T) / d) @ np.linalg.inv(K_left)
    return H / H[2, 2]

def warp_image(img_left, img_right, K_left, K_right, R, T):
    h, w = img_right.shape[:2]
    H = compute_homography(K_left, K_right, R, T)
    warped = cv2.warpPerspective(img_left, H, (w, h))
    return warped