#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
import dobot_Robot
import threading
from camera_realsenseD435 import RealsenseD435
from ultralytics import YOLO
# Callback function for clicking on OpenCV window
click_point_pix = ()
camera = RealsenseD435()

def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global camera, click_point_pix
        click_point_pix = (x, y)

        # Get click point in camera coordinates
        click_z = depth_image[y][x] * camera.get_depth_scale()
        click_x = np.multiply(x - intr_matrix[0][2], click_z / intr_matrix[0][0])
        click_y = np.multiply(y - intr_matrix[1][2], click_z / intr_matrix[1][1])
        if click_z == 0:
            return
        click_point = np.asarray([click_x, click_y, click_z])
        click_point.shape = (3, 1)

        image_to_arm = camera.image_to_arm()
        target_position = np.dot(image_to_arm[0:3, 0:3], click_point) + image_to_arm[0:3, 3:]
        target_position = target_position[0:3, 0]
        print(target_position)
        print(target_position.shape)
        move.MovJ(target_position[0], target_position[1], target_position[2], 0)

# Connect to the robot
dashboard, move, feed = dobot_Robot.connect_robot()
print("EnableRobot...")
dashboard.EnableRobot()
print("Enabled:)")
feed_thread = threading.Thread(target=dobot_Robot.get_feed, args=(feed,))
feed_thread.setDaemon(True)
feed_thread.start()
print("running...")
dobot_Robot.dobot_init(move, dashboard)

# Show color and depth frames
cv2.namedWindow('color')
cv2.setMouseCallback('color', mouseclick_callback)
cv2.namedWindow('depth')

while True:
    # Capture RGB and depth images
    rgb_image, depth_image, intr, intr_matrix, camera_distortion, aligned_depth_frame = camera.get_aligned_images()


    # model = YOLO("yolov8n-seg.pt")
    model = YOLO("yolov8n.pt")
    # model = YOLO("0703_yolov8n_300epochs_best.pt")

    results = model.track(source=rgb_image, show=False, stream=True)
    # results = model.predict(source="../Ring/test/video19.mpeg", show=False, stream=True)
    # results = model.track(
    #     source="../Ring/test/video18.mpeg", show=False, stream=True, save=True
    # )


    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
    )

    images = np.hstack((results, depth_colormap))

    cv2.namedWindow("yolov8", cv2.WINDOW_NORMAL)
    cv2.imshow("yolov8", images)


    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
dobot_Robot.dobot_init(move, dashboard)
dashboard.DisableRobot()


