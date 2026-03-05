import cv2
import numpy as np
from datetime import datetime
import os

WIDTH = 1280
HEIGTH = 720

WHAT_TO_DO = "RGBBB" # EVENT

def resize_rgb(rgb_image):
    h, w, _ = rgb_image.shape
    half_cropped_h = int((h - w * HEIGTH / WIDTH) / 2)
    rgb_image = rgb_image[half_cropped_h:-half_cropped_h, :, :]
    rgb_image = cv2.resize(rgb_image, (WIDTH, HEIGTH)) 
    return rgb_image

def rectify_stereo_pair(left_img, right_img, left_map1, left_map2, right_map1, right_map2):
    left_rectified = cv2.remap(left_img, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_img, right_map1, right_map2, cv2.INTER_LINEAR)
    return left_rectified, right_rectified

def test_rectification(left_img, right_img, left_map1, left_map2, right_map1, right_map2):

    # Rectify the images
    left_rect, right_rect = rectify_stereo_pair(left_img, right_img, left_map1, left_map2, right_map1, right_map2)
    
    # Create a side-by-side visualization
    vis = np.hstack((left_rect, right_rect))
    
    # Draw horizontal lines to verify rectification
    h, w = left_rect.shape[:2]
    for y in range(0, h, 50):
        cv2.line(vis, (0, y), (w*2, y), (0, 255, 0), 1)
    
    return left_rect, right_rect, vis

def rectify(K_left, D_left, K_right, D_right, R, T, img_left, img_right):
    assert img_left.shape == img_right.shape
    image_size = (img_left.shape[1], img_left.shape[0])

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_left, D_left, K_right, D_right, image_size, R, T, alpha=0
    )
    l_map1, l_map2 = cv2.initUndistortRectifyMap(
        K_left, D_left, R1, P1, image_size, cv2.CV_16SC2
    )
    r_map1, r_map2 = cv2.initUndistortRectifyMap(
        K_right, D_right, R2, P2, image_size, cv2.CV_16SC2
    )

    img_left_rect, img_right_rect, comparison = test_rectification(img_left, img_right, l_map1, l_map2, r_map1, r_map2)

    return img_left_rect, img_right_rect, comparison

if __name__ == "__main__":
    from all_images_rect import find_session
    FOLDER = find_session()
    rgb_event_calibration = os.path.join(FOLDER, "rgb_event_calibration.npz")
    rectification_folder = os.path.join(FOLDER, "rectification")

    os.makedirs(rectification_folder, exist_ok=True)
    current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

    rectification_folder = os.path.join(rectification_folder, current_time)
    os.makedirs(rectification_folder)

    with np.load(rgb_event_calibration) as data:
        rgb_K_left = data['rgb_K_left']
        rgb_D_left = data['rgb_D_left']
        rgb_K_right = data['rgb_K_right']
        rgb_D_right = data['rgb_D_right']
        event_K_left = data['event_K_left']
        event_D_left = data['event_D_left']
        event_K_right = data['event_K_right']
        event_D_right = data['event_D_right']
        rgb_R = data['rgb_R']
        rgb_T = data['rgb_T']
        event_R = data['event_R']
        event_T = data['event_T']
        rgb_event_R = data['rgb_event_R']
        rgb_event_T = data['rgb_event_T']


    if WHAT_TO_DO == "RGB":
        left_rgbs_folder = os.path.join(FOLDER, "rgb_left")
        right_rgbs_folder = os.path.join(FOLDER, "rgb_right")

        left_rgbs_files = os.listdir(left_rgbs_folder)
        right_rgbs_files = os.listdir(right_rgbs_folder)
        left_rgbs_files.sort()
        right_rgbs_files.sort()

        for i in range(len(left_rgbs_files)):
            if i < len(right_rgbs_files):
                left_rgb_file_name = left_rgbs_files[i]
                right_rgb_file_name = right_rgbs_files[i]
                left_rgb_path = os.path.join(FOLDER, "rgb_left", left_rgb_file_name)
                right_rgb_path = os.path.join(FOLDER, "rgb_right", right_rgb_file_name)


                # TMP
                # left_rgb_path = "/home/ubm/CVLab_Sensors/RGB_Event_Calibration/processed_data/2026_01_30_15_18_02/00024/rgb_left_corners.png"
                # right_rgb_path = "/home/ubm/CVLab_Sensors/RGB_Event_Calibration/processed_data/2026_01_30_15_18_02/00024/rgb_right_corners.png"

                # Read the RGB images
                left_rgb_image = resize_rgb(cv2.imread(left_rgb_path))
                right_rgb_image = resize_rgb(cv2.imread(right_rgb_path))

                before_rectification_rgb = np.concatenate([left_rgb_image, right_rgb_image], axis=1)

                cv2.imshow("Before Rectification", before_rectification_rgb)

                img_left_rect, img_right_rect, comparison = rectify(rgb_K_left, rgb_D_left, rgb_K_right, rgb_D_right, rgb_R, rgb_T, left_rgb_image, right_rgb_image)

                cv2.imshow("Comparison", comparison)

                cv2.waitKey(0)
    else:
        left_event_folder = os.path.join(FOLDER, "event_left")
        right_event_folder = os.path.join(FOLDER, "event_right")

        left_events_files = os.listdir(left_event_folder)
        right_events_files = os.listdir(right_event_folder)
        left_events_files.sort()
        right_events_files.sort()

        print("EVENT RECTIFICATION")

        for i in range(len(left_events_files)):
            if i < len(right_events_files):
                left_event_file_name = left_events_files[i]
                right_event_file_name = right_events_files[i]
                left_event_path = os.path.join(FOLDER, "event_left", left_event_file_name)
                right_event_path = os.path.join(FOLDER, "event_right", right_event_file_name)


                # TMP
                # left_rgb_path = "/home/ubm/CVLab_Sensors/RGB_Event_Calibration/processed_data/2026_01_30_15_18_02/00024/rgb_left_corners.png"
                # right_rgb_path = "/home/ubm/CVLab_Sensors/RGB_Event_Calibration/processed_data/2026_01_30_15_18_02/00024/rgb_right_corners.png"

                # Read the RGB images
                left_event_image = cv2.imread(left_event_path)
                right_event_image = cv2.imread(right_event_path)

                before_rectification_event = np.concatenate([left_event_image, right_event_image], axis=1)

                cv2.imshow("Before Rectification", before_rectification_event)

                img_left_rect, img_right_rect, comparison = rectify(event_K_left, event_D_left, event_K_right, event_D_right, event_R, event_T, left_event_image, right_event_image)

                cv2.imshow("Comparison", comparison)

                cv2.waitKey(0)









