# coding=utf-8
# grab depth camera image and do some computation, get the FPS and latency

# suppose you have run the realsense ROS 2 package to publish RGB-D image
# (go2) unitree@ubuntu:~/projects/tennis_project$ ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 camera_namespace:=go2 camera_name:=d435i
# you may also need depth_qos:=SENSOR_DATA color_qos:=SENSOR_DATA


# you can install cv2 with
# $ python3 -m pip install opencv-python --break-system-packages (Ubuntu 24.04 has Python 3.12 as system Python)
import cv2

import pyrealsense2 as rs2

# from ROS2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

# from the realsense-ros package
from realsense2_camera_msgs.msg import RGBD

import sys
import argparse
import numpy as np
from datetime import datetime
import time # for fps compute

from utils import image_resize
from utils import show_point_depth

parser = argparse.ArgumentParser()

parser.add_argument("--save_to_avi", default=None, help="save the visualization/rgb video to a avi file")
parser.add_argument("--image_size", default="640x480")
parser.add_argument("--rgbd_topic", default="/go2/d435i/rgbd")


def parse_intrinsics(cameraInfo):
    intrinsics = rs2.intrinsics()
    intrinsics.width = cameraInfo.width
    intrinsics.height = cameraInfo.height
    intrinsics.ppx = cameraInfo.k[2]
    intrinsics.ppy = cameraInfo.k[5]
    intrinsics.fx = cameraInfo.k[0]
    intrinsics.fy = cameraInfo.k[4]
    if cameraInfo.distortion_model == 'plumb_bob':
        intrinsics.model = rs2.distortion.brown_conrady
    elif cameraInfo.distortion_model == 'equidistant':
        intrinsics.model = rs2.distortion.kannala_brandt4
    intrinsics.coeffs = [i for i in cameraInfo.d]
    return intrinsics


class ImageListener(Node):
    def __init__(self, rgbd_topic, video_out=None, node_name="image_listener"):
        super().__init__(node_name)
        self.bridge = CvBridge()

        self.sub = self.create_subscription(RGBD, rgbd_topic, self.rgbdCallback, 1) # queue_size

        # define some global variables, used in callbacks
        self.start_timestamp = time.time() # in secs
        self.frame_count = 0

        self.intrinsics = None

        # for realsense
        # depth_value * depth_scale -> meters
        self.depth_scale = 0.001

        self.delay_time_list = []  # record the delay in average seconds

        # for video writer
        self.video_out = video_out


    def rgbdCallback(self, data):
        # here the data is parse as the RGBD message type

        """
        print(dir(data))
        ['SLOT_TYPES', '__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__getstate__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__slots__', '__str__', '__subclasshook__', '_check_fields', '_depth', '_depth_camera_info', '_fields_and_field_types', '_header', '_rgb', '_rgb_camera_info', 'depth', 'depth_camera_info', 'get_fields_and_field_types', 'header', 'rgb', 'rgb_camera_info']
        # see here: https://github.com/IntelRealSense/realsense-ros/blob/ros2-master/realsense2_camera_msgs/msg/RGBD.msg
        """

        color_data = data.rgb
        depth_data = data.depth

        # already numpy array
        color_image = self.bridge.imgmsg_to_cv2(color_data, color_data.encoding)
        # we need BGR to display properly
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        depth_image = self.bridge.imgmsg_to_cv2(depth_data, depth_data.encoding)

        # frame_count start from 1
        self.frame_count += 1

        if not self.intrinsics:
            # camera intrinsics only need to read once
            self.intrinsics = parse_intrinsics(data.rgb_camera_info)

        #(480, 640) (480, 640, 3)
        # [[2402 2393 2393 2384 2375 2375 2366 2366 2366 2357] # so the depth should be in mm
        # [ 640x480  p[308.003 247.865]  f[607.814 607.763]  Brown Conrady [0 0 0 0 0] ]
        #print(depth_image[250:255, 320:330])
        #print(depth_image.shape, color_image.shape)
        #print(self.intrinsics)

        # std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1728308261, nanosec=500469238), frame_id='camera_rgbd_optical_frame')
        #print(data.header)

        # for visualization

        # showing two points' depth
        point1 = (200, 200)  # (y, x)
        point2 = (240, 320)

        color_image, depth1 = show_point_depth(point1, depth_image, color_image)
        color_image, depth2 = show_point_depth(point2, depth_image, color_image)
        # mm to meters
        depth1 = depth1 * self.depth_scale
        depth2 = depth2 * self.depth_scale

        # rs2_deproject_pixel_to_point takes pixel (x, y)
        # outputs (x, y, z), the coordinates are in meters
        #   [0,0,0] is the center of the camera, 相机朝向的右边是正x，下边为正y, 朝向是正z
        #   See this doc for coordinate system
        #   https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0?fbclid=IwAR3gogVZe824YUps88Dzp02AN_XzEm1BDb0UbmzfoYvn1qDFb7KzbIz9twU#point-coordinates
        # 理解此函数，需要知道camera model，perspective projection, geometric computer vision
        # 也就是说3D世界的坐标如何与相机上的像素坐标互相转换的
        point1_3d = rs2.rs2_deproject_pixel_to_point(self.intrinsics, (point1[1], point1[0]), depth1)
        point2_3d = rs2.rs2_deproject_pixel_to_point(self.intrinsics, (point2[1], point2[0]), depth2)

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

        # 2*image_width, image_height
        #print_once("image shape: %s" % list(image.shape[:2]))

        # show the fps in the visualization
        current_timestamp = time.time()
        fps = self.frame_count / (current_timestamp - self.start_timestamp)
        image = cv2.putText(
            image, "FPS: %d" % int(fps),
            (10, 330), cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1, color=(0, 0, 255), thickness=2)

        # put a timestamp for the frame for possible synchronization
        # and a frame index to look up depth data
        current_time_str = datetime.fromtimestamp(current_timestamp).strftime('%Y-%m-%d %H:%M:%S')

        # combine the nano seconds to be more precise
        data_timestamp = data.header.stamp.sec + float(data.header.stamp.nanosec) * 1e-9
        #data_time_str = datetime.fromtimestamp(data_timestamp).strftime('%Y-%m-%d %H:%M:%S')

        delay_time = abs(current_timestamp - data_timestamp)
        self.delay_time_list.append(delay_time)
        current_delay = np.mean(self.delay_time_list)

        image = cv2.putText(
            image, "#%d: %s (delay: %.4fs)" % (self.frame_count, current_time_str, current_delay),
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1, color=(0, 0, 255), thickness=2)

        if self.video_out is not None:

            self.video_out.write(image)

        # Show the image
        cv2.imshow('RGB and Depth Stream', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit



if __name__ == "__main__":
    args = parser.parse_args()

    image_width, image_height = [int(x) for x in args.image_size.split("x")]


    print("Now showing the camera stream. press Q to exit.")

    listener = None
    try:
        rclpy.init()

        video_out = None
        if args.save_to_avi is not None:

            # cannot save to mp4 file, due to liscensing problem, need to compile opencv from source
            print("saving to avi video %s..." % args.save_to_avi)
            fourcc = cv2.VideoWriter_fourcc(*"XVID")

            # the visualization video size
            width_height = (image_width*2, image_height)

            video_out = cv2.VideoWriter(args.save_to_avi, fourcc, 30.0, width_height)

        listener = ImageListener(args.rgbd_topic, video_out)

        rclpy.spin(listener)  # this will be blocked


    finally:

        listener.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        if args.save_to_avi is not None:
            video_out.release()

        # print out some stats
        if listener:
            end_timestamp = time.time()
            run_time = (end_timestamp - listener.start_timestamp)
            fps = listener.frame_count / run_time
            avg_delay = np.mean(listener.delay_time_list)

            print("total run time %.2f secs, FPS %.2f, avg_delay %.3f secs" % (
                run_time, fps, avg_delay))

