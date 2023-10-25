import threading
import time
import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import dobot_Robot


# camera config
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

# get image
def get_aligned_images( ):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # intelrealsense intrinsics
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    # to ndarray for opencv 
    intr_matrix = np.array([
        [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
    ])
    # depth image-16 bit
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    # depth image-8 bit
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
    pos = np.where(depth_image_8bit == 0)
    depth_image_8bit[pos] = 255
    # rgb
    color_image = np.asanyarray(color_frame.get_data())
    # return: rgb，depth，intrinsics ,intrinsics_matrix, Camera distortion(intr.coeffs), aligned_depth_frame
    return color_image, depth_image,intr, intr_matrix, np.array(intr.coeffs),aligned_depth_frame


def center_aruco():
    rgb, depth, intr,intr_matrix, intr_coeffs,aligned_depth_frame = get_aligned_images()
    # aruco
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    # create detector parameters
    parameters = aruco.DetectorParameters_create()
    # lists of ids and the corners beloning to each id
    corners, ids, rejected_img_points = aruco.detectMarkers(rgb, aruco_dict, parameters=parameters,
                                                            cameraMatrix=intr_matrix, distCoeff=intr_coeffs)
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
                           [236, 24, 29, 0], [169, 30, 53, 0],
                           [262, 41, 7, 0], [221, 4, 67, 0],
                           [214, -29, 125, 0], [237, -38, -12, 0]]
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
    dobot_Robot.dobot_init()


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
    dobot_Robot.dobot_init()

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
    pipeline.stop()
    time.sleep(1)
    dashboard.DisableRobot()
