# note on learning ROS 2

1. Basic concept
```
    # Ubuntu 24.04 -> ROS 2 jazzy
    # tutorial: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

    # sourcing adds env variable like PATH and PYTHONPATH and LD_LIBRARY_PATH
        $ source /opt/ros/jazzy/setup.bash
        $ printenv | grep -i ROS

    # ROS_DOMAIN_ID defaults to be 0, this is to prevent other ROS 2 application affect

    # nodes - one launch file might get one node, one node could create multiple topics

        $ ros2 node list

        $ ros2 node info /camera/camera
            # gets the current subscribers, publishers, service and others of this node

    # topics - messages to send and receive, the publisher will continuously send info
        # a one-way street

        # rqt_graph can show the current ros2 graph

        $ ros2 topic list [-t to show message type]

        $ ros2 topic echo /camera/camera/color/camera_info
            # show the message

        # check topic info, message type
            (base) junweil@precognition-laptop4:~$ ros2 topic info /camera/camera/color/image_raw
                Type: sensor_msgs/msg/Image
                Publisher count: 1
                Subscription count: 0

            # check message detail info

                $ ros2 interface show sensor_msgs/msg/Image

        # check the topic Hz
            $ ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw

    # service - call-and-response, service server, can provide service to service-client
        # only when client ask, the service-server will do something and response

    # parameters - node has parameters that can be set

    # actions - goal/feedback/result - built on topics and services

    # you can do ros2 run to start a node
    # or you can write a launch.py file, and use ros2 launch to start multiple nodes

    # record the topic data
        $ ros2 bag record -o <bag_file_name> /turtle1/cmd_vel /turtle1/pose [topic3 ...]

        # see the recorded bag file
            $ ros2 bag info <bag_file_name>

    # The Idea of quality of service (QoS)
        # (when no subscriber to a topic, the message will not be sent at all)
        # the default setting will make sure publisher receive ack from the consumer
        # set to best_effort on a lossy network: https://docs.ros.org/en/jazzy/Tutorials/Demos/Quality-of-Service.html

        # https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
            # can be as reliable as TCP, or as best-effort as UDP

        # a set of QoS policies -> a QoS profile -> can be set for publishers, subscriptions, server/client
            # in realsense,  the QoS is set for the topic/stream_type: depth/color
            # policy
                # history -> keep_last or Keep_all
                    # depth=1 -> keep last N samples
                # reliability -> reliable / best_effort

            # example for sending depth image over wireless network

                from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

                depth_image_qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Prioritize speed over reliability
                    durability=QoSDurabilityPolicy.VOLATILE,       # No need to store old frames
                    history=QoSHistoryPolicy.KEEP_LAST,            # Keep only the latest frame
                    depth=1                                        # Discard all but the most recent frame
                )
```

2. Writing a publisher and subscriber
```
    1. workspace - containing ROS 2 package
        Use colon to build: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#new-directory

    2. write a publisher and subscriber
        https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

        in defining the Node
            Node name need to be called in Super().__init__(name)

            from std_msgs.msg import String # this is the built-in data type
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            # 10 is the queue size, String is a built-in message type

            # this function will be called every timer_period seconds, decide how fast the node is publishing
            self.timer = self.create_timer(timer_period, self.timer_callback)

            def self.timer_callback(self):
                self.publisher.publish(...)

        in Subscriber node:
            self.subscription = self.create_subscription(
                String,  # the message type need to be predefined.
                'topic',
                self.listener_callback,
                10)
            10 is the queue. The callback will be executed once it receives message

        in Main():
            rclpy.init()
            minimal_publisher = MinimalPublisher()
            rclpy.spin(minimal_publisher)
            ...
            # done
            minimal_publisher.destroy_node()
            rclpy.shutdown()

        # parsing the message
            # the built-in message types package are for example:
                from sensor_msgs.msg import Image as msg_Image
                from sensor_msgs.msg import Imu as msg_Imu
                from std_msgs.msg import String
            # you install some package from apt like: sudo apt install ros-jazzy-realsense2-*
                # then you can directly import the message from that package like this

                    # from realsense2_camera_msgs.msg import RGBD
                        # https://github.com/IntelRealSense/realsense-ros/tree/ros2-master/realsense2_camera_msgs
                        # installed here: /opt/ros/jazzy/lib/python3.12/site-packages/realsense2_camera_msgs/

                    # the ros2 package is called realsense2_camera
                            in here realsense-ros/tree/ros2-master/realsense2_camera
                        # the message is a separate package?
                            realsense2_camera_msgs
                            in here realsense-ros/tree/ros2-master/realsense2_camera_msgs

```
