import threading
import time
import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import dobot_Robot
from camera_realsenseD435 import RealsenseD435

def center_aruco():
    camera=RealsenseD435()
    rgb, depth, intr,intr_matrix, intr_coeffs,aligned_depth_frame = camera.get_aligned_images()
    # aruco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    # create detector parameters
    parameters = aruco.DetectorParameters()
    detector= aruco.ArucoDetector(aruco_dict,parameters)
    # lists of ids and the corners beloning to each id
    corners, ids, rejected_img_points = detector.detectMarkers(rgb)
    if len(corners) != 0:
        point = np.average(corners[0][0], axis=0)
        depth = aligned_depth_frame.get_distance(point[0], point[1])
        point = np.append(point, depth)
        if depth != 0:
            global center
            global color_image
            #                     print("center:%f %f, depth:%f m" %(point[0], point[1], point[2]))
            x = point[0]
            y = point[1]
            z = point[2]
            ## see rs2 document:
            ## https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates
            ## and example: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates
            x, y, z = rs.rs2_deproject_pixel_to_point(intr, [x, y], z)
            center = [x, y, z]
            # print("center is ", center)
            #                     print(center)
            color_image = aruco.drawDetectedMarkers(rgb, corners)

    else:
        center, color_image = None, None
        print("not found aruco!")
    return center, color_image



if __name__ == "__main__":
    n = 0
    i = 0
    default_cali_points = [[209, -140, 75, 0], [205, -92, 64, 0],
                           [236, 24, 29, 0], [269, 30, 53, 0],
                           [300, 0, 0, 0], [221, 4, 67, 0]]
    np_cali_points = np.array(default_cali_points)
    arm_cord = np.column_stack(
        (np_cali_points[:, 0:3], np.ones(np_cali_points.shape[0]).T)).T
    centers = np.ones(arm_cord.shape)

    dashboard, move, feed = dobot_Robot.connect_robot()
    print("EnableRobot...")
    dashboard.EnableRobot()
    print("Enabled:)")
    feed_thread = threading.Thread(target=dobot_Robot.get_feed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()
    print("running...")
    dobot_Robot.dobot_init(move,dashboard)


    for ind, pt in enumerate(default_cali_points):
        print("Current points:", pt)
        move.MovL(pt[0], pt[1], pt[2], pt[3])
        dobot_Robot.wait_arrive(pt)
        time.sleep(3)
        center, color_image = center_aruco()
        if center is not None:
            centers[0:3, ind] = center
            print("centers is ", center)
        else:
            print("no aruco!")

    image_to_arm = np.dot(arm_cord, np.linalg.pinv(centers))
    arm_to_image = np.linalg.pinv(image_to_arm)

    np.savetxt('./Dobot-MG400/image_to_arm.txt', image_to_arm, delimiter=' ')
    np.savetxt('./Dobot-MG400/arm_to_image.txt', arm_to_image, delimiter=' ')

    print('txt saved.')


        
    dobot_Robot.dobot_init(move,dashboard)


    print("Finished")
    print("Image to arm transform:\n", image_to_arm)
    print("Arm to Image transform:\n", arm_to_image)


    print("Sanity Test:")

    print("-------------------")
    print("Image_to_Arm")
    print("-------------------")
    for ind, pt in enumerate(centers.T):
        print("Expected:", default_cali_points[ind][0:3])
        print("Result:", np.dot(image_to_arm, np.array(pt))[0:3])

    print("-------------------")
    print("Arm_to_Image")
    print("-------------------")
    for ind, pt in enumerate(default_cali_points):
        print("Expected:", centers.T[ind][0:3])
        pt[3] = 1
        print("Result:", np.dot(arm_to_image, np.array(pt))[0:3])

    cv2.destroyAllWindows()
    #pipeline.stop()
    #pipeline.stop()
    time.sleep(1)
