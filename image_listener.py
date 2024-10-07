# coding=utf-8
# grab depth camera image and do some computation, get the FPS and latency

# suppose you have run the realsense ROS 2 package to publish RGB-D image
# junweil@precognition-laptop4:~/projects/realsense$ ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true camera_namespace:=go2 camera_name:=d435i


# you can install cv2 with
# $ python3 -m pip install opencv-python --break-system-packages (Ubuntu 24.04 has Python 3.12 as system Python)
import cv2

import pyrealsense2 as rs2

# from ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

# from the realsense-ros package
from realsense2_camera_msgs.msg import RGBD

import sys
import argparse
import numpy as np
import datetime
import time # for fps compute

from utils import image_resize
from utils import print_once
from utils import show_point_depth

parser = argparse.ArgumentParser()

parser.add_argument("--camera_type", default="realsense")
parser.add_argument("--save_to_avi", default=None, help="save the visualization/rgb video to a avi file")
parser.add_argument("--image_size", default="640x480")
parser.add_argument("--rgbd_topic", default="/go2/d435i/rgbd")

class ImageListener(Node):
    def __init__(self, rgbd_topic, node_name="image_listener"):
        super().__init__(node_name)
        self.bridge = CvBridge()

        self.sub = self.create_subscription(RGBD, rgbd_topic, self.rgbdCallback, 1) # queue_size

        # define some global variables, used in callbacks
        self.start_time = time.time()
        self.frame_count = 0

        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

    def rgbdCallback(self, data):
        # here the data is parse as the RGBD message type
        print(dir(data))
        raise Exception("done.")
        return

        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        # pick one pixel among all the pixels with the closest range:
        indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
        pix = (indices[1], indices[0])
        self.pix = pix
        line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])

        if self.intrinsics:
            depth = cv_image[pix[1], pix[0]]
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
            line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
        if (not self.pix_grade is None):
            line += ' Grade: %2d' % self.pix_grade
        line += '\r'
        sys.stdout.write(line)
        sys.stdout.flush()

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # frame_count start from 1
        frame_count += 1

        # junwei: the color_intrin and depth_intrin are the same as they are aligned.
        #### 获取相机参数 ####
        aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
        aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
        # [ 640x480  p[325.217 238.38]  f[385.38 384.848]  Inverse Brown Conrady [-0.0565123 0.067672 0.000208852 0.000719325 -0.0218305] ]
        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参

        #print(depth_intrin, color_intrin)

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # (720, 1280, 3), (720, 1280)
        #print(color_image.shape, depth_image.shape)
        #print(depth_image[240, 320]) # 单位：毫米


        # for visualization

        # showing two points' depth
        point1 = (400, 400)  # (y, x)
        point2 = (480, 640)

        color_image, depth1 = show_point_depth(point1, depth_image, color_image)
        color_image, depth2 = show_point_depth(point2, depth_image, color_image)
        # mm to meters
        depth1 = depth1 * depth_scale
        depth2 = depth2 * depth_scale

        # rs2_deproject_pixel_to_point takes pixel (x, y)
        # outputs (x, y, z), the coordinates are in meters
        #   [0,0,0] is the center of the camera, 相机朝向的右边是正x，下边为正y, 朝向是正z
        #   See this doc for coordinate system
        #   https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0?fbclid=IwAR3gogVZe824YUps88Dzp02AN_XzEm1BDb0UbmzfoYvn1qDFb7KzbIz9twU#point-coordinates
        # 理解此函数，需要知道camera model，perspective projection, geometric computer vision
        # 也就是说3D世界的坐标如何与相机上的像素坐标互相转换的
        point1_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, (point1[1], point1[0]), depth1)
        point2_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, (point2[1], point2[0]), depth2)

        # 计算这两点的实际距离
        #print(point1_3d, point2_3d)
        dist_between_point1_point2 = np.linalg.norm(np.array(point1_3d) - np.array(point2_3d))

        mid_point_xy = ( int((point2[1] + point1[1])/2.), int((point2[0] + point1[0])/2.) + 100)
        color_image = cv2.putText(
            color_image, "dist 1to2: %.2f meters" % dist_between_point1_point2,
            mid_point_xy, cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1, color=(0, 0, 255), thickness=2)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        image = np.hstack((color_image, depth_colormap))

        image = image_resize(image, width=1280, height=None)

        print_once("image shape: %s" % list(image.shape[:2]))


        # put a timestamp for the frame for possible synchronization
        # and a frame index to look up depth data
        date_time = str(datetime.datetime.now())
        image = cv2.putText(
            image, "#%d: %s" % (frame_count, date_time),
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1, color=(0, 0, 255), thickness=2)

        # show the fps in the visualization
        current_time = time.time()
        fps = frame_count / (current_time - start_time)
        image = cv2.putText(
            image, "FPS: %d" % int(fps),
            (10, 330), cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1, color=(0, 0, 255), thickness=2)

        if args.save_to_avi is not None:

            out.write(image)

        # Show the image
        cv2.imshow('RGB and Depth Stream', image)



if __name__ == "__main__":
    args = parser.parse_args()

    image_width, image_height = [int(x) for x in args.image_size.split("x")]

    # depth_value * depth_scale -> meters
    # depth_scale # 0.001


    print("Now showing the camera stream. press Q to exit.")

    try:
        rclpy.init()
        listener = ImageListener(args.rgbd_topic)
        rclpy.spin(listener)

        if args.save_to_avi is not None:

            # cannot save to mp4 file, due to liscensing problem, need to compile opencv from source
            print("saving to avi video %s..." % args.save_to_avi)
            fourcc = cv2.VideoWriter_fourcc(*"XVID")

            # the visualization video size
            width_height = (image_width*2, image_height)

            out = cv2.VideoWriter(args.save_to_avi, fourcc, 30.0, width_height)

        while True:



            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    finally:

        listener.destroy_node()
        rclpy.shutdown()
        if args.save_to_avi is not None:
            out.release()
        cv2.destroyAllWindows()
