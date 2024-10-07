# note for using Depth Cameras in ROS 2

1. Realsense on Ubuntu 24.04
+ Installation
```
    # https://github.com/IntelRealSense/realsense-ros/tree/ros2-development

    # first, ROS 2 does not work with conda environment, so we need to better have python 3.11
        # ubuntu 24.04 gets you python 3.12

        # laptop4 originally has python3.11 in conda and python3.12
        # ROS2 says it cannot work with anaconda
            https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html

        # you cannot change the system python version since apt-get relies on it

    1. Install ROS2 Jazzy for Ubuntu 24 (conda deactivate, with system python3.12)
        https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

        # uninstall
            sudo apt remove ~nros-jazzy-* && sudo apt autoremove

        $ sudo apt install software-properties-common
        $ sudo add-apt-repository universe
        ...

        $ sudo apt install ros-jazzy-desktop

        1.2. Setup Environment
            # everytime you need to source the ros2 stuff

            $ source /opt/ros/jazzy/setup.bash

        1.3. Test
            Screen 1: send some info as topic using C++ API
                $ source /opt/ros/jazzy/setup.bash
                $ ros2 run demo_nodes_cpp talker

            Screen 2: receive the message using Python API
                $ source /opt/ros/jazzy/setup.bash
                $ ros2 run demo_nodes_py listener

        # check your ROS2 is using the python version you normally use
            $ ls /opt/ros/jazzy/lib/python3.12/

            # the python package for ROS 2 is rclpy
                # you will need to conda deactivate to import rclpy
                >>> rclpy.__path__
                ['/opt/ros/jazzy/lib/python3.12/site-packages/rclpy']

    2. Install Realsense SDK 2.0
        # need to build from source on Ubuntu 24.04:
            https://github.com/IntelRealSense/realsense-ros/tree/ros2-development?tab=readme-ov-file#option-3-build-from-source

                https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

        # clone the github repo librealsense and install

            # Note: The shared object will be installed in /usr/local/lib, header files in /usr/local/include.
            # The binary demos, tutorials and test files will be copied into /usr/local/bin

            # install - also build the python 3.12 wrapper
                #  install pyrealsense wrapper, for python3.12 we need to build from source\
                    # https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python#building-from-source

                $ mkdir build && cd build
                $ cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3
                $ sudo make uninstall && make clean && make -j8 && sudo make install

                    # pyrealsense2 is installed to '/usr/local/lib/python3.12/dist-packages/pyrealsense2'

                    # copy librealsense/build/Release/*.so* lib to the system python, add __init__.py file

                        junweil@precognition-laptop4:~$ sudo cp ./projects/realsense/librealsense/build/Release/pyrealsense2.cpython-312-x86_64-linux-gnu.so* /usr/local/lib/python3.12/dist-packages/pyrealsense2/

                        junweil@precognition-laptop4:~/projects/realsense/librealsense/build/Release$ sudo vi /usr/local/lib/python3.12/dist-packages/pyrealsense2/__init__.py

                            # py libs (pyd/so) should be copied to pyrealsense2 folder
                            from .pyrealsense2 import *

                        $ ls /usr/local/lib/python3.12/dist-packages/pyrealsense2
                        __init__.py                                          pyrsutils.cpython-312-x86_64-linux-gnu.so
                        pyrealsense2.cpython-312-x86_64-linux-gnu.so         pyrsutils.cpython-312-x86_64-linux-gnu.so.2.55
                        pyrealsense2.cpython-312-x86_64-linux-gnu.so.2.55    pyrsutils.cpython-312-x86_64-linux-gnu.so.2.55.1
                        pyrealsense2.cpython-312-x86_64-linux-gnu.so.2.55.1

                # if set -DPYTHON_EXECUTABLE=/home/junweil/anaconda3/bin/python
                    # pyrealsense2 is installed to /home/junweil/anaconda3/lib/python3.12/site-packages/pyrealsense2

                    # we need to install this for system python


        # test:
            realsense-viewer  # check latency qualitatively
            python - import pyrealsense2

    3. Install Realsense ROS package
        # install from source to have a latency test tool - nope, the latency is running locally, not useful
        # install from apt install
            # https://github.com/IntelRealSense/realsense-ros/tree/ros2-development?tab=readme-ov-file#option-1-install-debian-package-from-ros-servers-foxy-eol-distro-is-not-supported-by-this-option

            $ sudo apt install ros-jazzy-realsense2-*

            # this will install the realsense ros stuff to
                # ['/opt/ros/jazzy']

    #### note we may need to conda deactivate to use ROS python

```
+ Testing
```
    # Realsense ROS 2 documentation: https://dev.intelrealsense.com/docs/ros2-wrapper
        # code: https://github.com/IntelRealSense/realsense-ros

    1. running the RGBD Topic in one process

        # https://github.com/IntelRealSense/realsense-ros/tree/ros2-development?tab=readme-ov-file#rgbd-topic

        $ conda deactivate
        $ source /opt/ros/jazzy/setup.bash

        junweil@precognition-laptop4:~/projects/realsense$ ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true camera_namespace:=go2 camera_name:=d435i

            # depth_qos:=SENSOR_DATA color_qos:=SENSOR_DATA
                # so this can be best_effort, higher frame rate?

            # depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30

            # camera_namespace:=robot1 camera_name:=D455_1
            # so you get /robot1/D455_1 node

            # to see this code: https://github.com/IntelRealSense/realsense-ros
                # realsense2_camera/launch/rs_launch.py

        $ ros2 topic list
            /camera/camera/aligned_depth_to_color/camera_info
            /camera/camera/aligned_depth_to_color/image_raw
            /camera/camera/color/camera_info
            /camera/camera/color/image_raw
            /camera/camera/color/metadata
            /camera/camera/depth/camera_info
            /camera/camera/depth/image_rect_raw
            /camera/camera/depth/metadata
            /camera/camera/extrinsics/depth_to_color
            /camera/camera/rgbd
            /parameter_events
            /rosout
            /tf_static

        # check the Hz
            $ ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
            2 Hz - 25 Hz??
            $ ros2 topic hz /camera/camera/color/camera_info
            30 Hz
            $ ros2 topic hz /camera/camera/depth/image_rect_raw
            22 Hz
            $ ros2 topic hz /camera/camera/color/image_raw
            2-12 Hz

            # making it faster over ROS?
                https://github.com/IntelRealSense/librealsense/issues/10730

    2. listen to this message and visualize

        # need to install a package (first do $ conda deactivate)
            $ python3 -m pip install numpy-quaternion --break-system-packages

        2.1 use the example code

            junweil@precognition-laptop4:~/projects/realsense/realsense-ros/realsense2_camera/scripts$ python3 rs2_listener.py /camera/camera/color/image_raw

                # https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_camera/scripts/rs2_listener.py

            # show center depth

            junweil@precognition-laptop4:~/projects/realsense/realsense-ros/realsense2_camera/scripts$ python3 show_center_depth.py



```


2. Realsense on Jetsons

3. Orbbec on Ubutn 24.04
```
https://github.com/orbbec/OrbbecSDK_ROS2
```
