# ROS Line Following Robot

![Line Follower Robot](/assets/images/robot-2.JPEG)

This project is a small line-following robot. It can detect and follow tape lines on my floor! 
The robot runs ROS on a Raspberry Pi, uses OpenCV to detect lines, and an Arduino Pro Micro to control
differential steering.

I built this small project to sharpen my ROS and OpenCV skills. Your mileage may vary; the OpenCV code
likely will not work in a different lighting environment. The perception system won't generalize since it relies on the line being significantly _lighter_ than the floor, which works in my workspace.

## Architecture

```

   ┌─────────────────────────────────────────┐   ┌─────────────────────────────────────────────────────────────────┐
   │                                         │   │                                                                 │
   │   ROBOT HARDWARE                        │   │   ROS NODES                                                     │
   │                                         │   │                                                                 │
   │   ┌──────────────┐                      │   │   ┌────────────────────────┐             ┌──────────────────┐   │
   │   │              │   Raw                │   │   │                        │  Processed  │                  │   │
   │   │ Raspberry Pi │   Image              │   │   │ Image Preparation      │  Image      │  Line Detection  │   │
   │   │    Camera    ├──────────────────────┼───┼───► Threshold,             ├─────────────►  Path Planning   │   │
   │   │              │                      │   │   │ Perspective-Correction │             │                  │   │
   │   └──────────────┘                      │   │   │                        │             │                  │   │
   │                                         │   │   └────────────────────────┘             └─────────┬────────┘   │
   │                                         │   │                                                    │            │
   │   ┌──────────────┐                      │   │   ┌────────────────────────┐                       │            │
   │   │              │  Speed/Heading       │   │   │                        │  Speed/Heading        │            │
   │   │ Arduino Pro  │  Command             │   │   │  Serial Communication  │  Command              │            │
   │   │ Micro        ◄──────────────────────┼───┼───┤  Node                  ◄───────────────────────┘            │
   │   │              │                      │   │   │                        │                                    │
   │   └──────┬───────┘                      │   │   └────────────────────────┘                                    │
   │          │                              │   │                                                                 │
   │   Motor  │                              │   └─────────────────────────────────────────────────────────────────┘
   │   Commands                              │
   │          │            ┌──────────────┐  │
   │   ┌──────▼───────┐    │              │  │
   │   │              ├────► Left Motor   │  │
   │   │              │    │              │  │
   │   │ Dual Motor   │    └──────────────┘  │
   │   │ Controller   │                      │
   │   │              │    ┌──────────────┐  │
   │   │              ├────►              │  │
   │   └──────────────┘    │ Right Motor  │  │
   │                       │              │  │
   │                       └──────────────┘  │
   │                                         │
   └─────────────────────────────────────────┘
```


1. Raspberry Pi Camera

An image is taken in from a Raspberry Pi Camera mounted on the robot.

2. Perspective Correction

The robot has a fixed camera angle and distance off the ground, and assumes
that the ground is flat. These limitations allow the software to perform a 
fixed perspective correction on the input image. This assists in detecting and following the lines in later steps.

3. Hough Transform

The image is passed through a Hough Transform filter, which generates a list of lines that were detected in the image. An algorithm groups the lines by angle, and
assigns relative weights based on the line length and number of detections.

4. Path Planning



## Development Notes

### Prerequisites
To run this project, you must have a ROS Noetic installation setup on a Raspberry Pi. This repository should be checked out into a Catkin workspace. For more information, see the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials).

You must also have the following dependencies installed: 
- Python 3 (`sudo apt install python3.8`)
- OpenCV (`pip install opencv-python && sudo apt install python3-pip`)
- rosserial_python (`sudo apt install ros-noetic-rosserial-python`)
- robot_upstart (`sudo apt install ros-noetic-robot-upstart`)
- raspicam_node (See https://github.com/UbiquityRobotics/raspicam_node for build/install instructions)

### Starting the Project
To run the entire project, use the roslaunch file in this repository:
`roslaunch line_follower line_follower.launch`

### Remote ROS

To connect to the remote ROS instance, set the following environment variables:
```
export ROS_MASTER_URI=http://linefollower:11311
export ROS_IP=linefollower
```

Start the visualization:
```
rosrun rviz rviz -d ~/linefollower/catkin_ws/src/ros-line-follower-robot/rviz/line_follower.rviz
```

### Starting/Stopping the Background Service

The entire stack can be launched in the background using `robot_upstart`.

To install the background service:
```
rosrun 
    robot_upstart install 
    --job linefollower
    --setup /home/ubuntu/catkin_ws/devel/setup.sh
    --logdir /var/log/upstart
    --symlink line_follower/launch/line_follower.launch
sudo systemctl daemon-reload
```

To start the background service:
```
sudo systemctl enable linefollower
sudo systemctl start linefollower
```

To stop the background service:
```
sudo systemctl stop linefollower
sudo systemctl disable linefollower
```