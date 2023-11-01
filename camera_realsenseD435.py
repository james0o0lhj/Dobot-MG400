import numpy as np
import pyrealsense2 as rs
import cv2

class RealsenseD435(object):
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

   # get image
    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
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

        # rgb
        color_image = np.asanyarray(color_frame.get_data())
        # return: rgb，depth，intrinsics ,intrinsics_matrix, Camera distortion(intr.coeffs), aligned_depth_frame
        return color_image, depth_image,intr, intr_matrix, np.array(intr.coeffs),aligned_depth_frame
    
    def get_depth_scale(self):
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        return depth_scale
        
    def image_to_arm(self):
        image_to_arm=np.loadtxt('./Dobot-MG400/image_to_arm.txt', delimiter=' ')   #eye to hand calib
        return image_to_arm

if __name__ == '__main__':
    camera = RealsenseD435()

    cv2.namedWindow('color')
    cv2.namedWindow('depth')

    while True:
        rgb_image, depth_image, _, _, _, _ = camera.get_aligned_images()  

        cv2.imshow('color', rgb_image)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )
        cv2.imshow('depth', depth_colormap)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

