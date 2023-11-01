#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
import dobot_Robot
import camera_realsenseD435 as camera
import threading   

dashboard, move, feed = dobot_Robot.connect_robot()
print("EnableRobot...")
dashboard.EnableRobot()
print("Enabled:)")
feed_thread = threading.Thread(target=dobot_Robot.get_feed, args=(feed,))
feed_thread.setDaemon(True)
feed_thread.start()
print("running...")
dobot_Robot.dobot_init()

# Callback function for clicking on OpenCV window
click_point_pix = ()
rgb_image, depth_image, intr,intr_matrix, camera_distortion,aligned_depth_frame = camera.get_aligned_images()

def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global camera, click_point_pix
        click_point_pix = (x,y)

        # Get click point in camera coordinates
        click_z = depth_image[y][x] * camera.get_depth_scale()
        click_x = np.multiply(x-camera.intr_matrix[0][2],click_z/camera.intr_matrix[0][0])
        click_y = np.multiply(y-camera.intr_matrix[1][2],click_z/camera.intr_matrix[1][1])
        if click_z == 0:
            return
        click_point = np.asarray([click_x,click_y,click_z])
        click_point.shape = (3,1)

        # Convert camera to robot coordinates
        # camera2robot = np.linalg.inv(robot.cam_pose)
        with open("image_to_arm.txt", "r") as image_file:
            image_to_arm = image_file.read()
        target_position = np.dot(image_to_arm, click_point)

        #target_position = target_position[0:3,0]
        print(target_position)
        print(target_position.shape)
        move.MovJ([target_position[0],target_position[1],target_position[2]])


# Show color and depth frames
cv2.namedWindow('color')
cv2.setMouseCallback('color', mouseclick_callback)
cv2.namedWindow('depth')

while True:
    bgr_data = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    if len(click_point_pix) != 0:
        bgr_data = cv2.circle(bgr_data, click_point_pix, 7, (0,0,255), 2)
    cv2.imshow('color', bgr_data)
    cv2.imshow('depth', depth_image)
    
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
