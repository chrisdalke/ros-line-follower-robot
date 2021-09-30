# ROS Line Following Robot

![Line Follower Robot](/assets/images/robot-2.JPEG)

This project is a small line-following robot. It can detect and follow tape lines on my floor! 
The robot runs ROS on a Raspberry Pi, uses OpenCV to detect lines, and an Arduino Pro Micro to control
differential steering.

I built this small project to sharpen my ROS and OpenCV skills. Your mileage may vary; the OpenCV code
likely will not work in a different lighting environment. The perception system won't generalize since it relies on the line being significantly _lighter_ than the floor, which works in my workspace.

## Architecture

```
┌────────────────────────────────┐ ┌─────────────────────────────────────────────────────────┐
│ ROBOT HARDWARE                 │ │ ROS NODES                                               │
│ ┌──────────────┐ Raw           │ │ ┌────────────────────────┐ Processed ┌────────────────┐ │
│ │ Raspberry Pi │ Image         │ │ │ Image Preparation      │ Image     │ Line Detection │ │
│ │    Camera    ├───────────────┼─┼─► Threshold,             ├───────────► Path Planning  │ │
│ └──────────────┘               │ │ │ Perspective-Correction │           └───────┬────────┘ │
│                                │ │ └────────────────────────┘                   │          │
│ ┌──────────────┐ Speed/Angle   │ │                                              │          │
│ │ Arduino Pro  │ Command       │ │ ┌────────────────────────┐  Speed/Angle      │          │
│ │ Micro        ◄───────────────┼─┼─┤ Serial Communication   │  Command          │          │
│ └──────┬───────┘               │ │ │ Node                   ◄───────────────────┘          │
│        │ Motor                 │ │ └────────────────────────┘                              │
│        │ Command               │ │                                                         │
│ ┌──────▼─────────────────┐     │ └─────────────────────────────────────────────────────────┘
│ │       Dual Motor       │     │
│ │       Controller       │     │
│ └─────┬────────────┬─────┘     │
│       │            │           │
│ ┌─────▼────┐ ┌─────▼─────┐     │
│ │Left Motor│ │Right Motor│     │
│ └──────────┘ └───────────┘     │
│                                │
└────────────────────────────────┘
```
The robot architecture is a unidirectional pipeline. First, sensor data is fed from the Raspberry Pi camera, and through image processing which performs a perspective correction and contrast/threshold image adjustment to facilitate line detection.

Once the image is processed, a line detection algorithm is run to find the approximate direction of the line to follow.
The code uses the relative position and angle of the line to compute a target speed/angle that will track the line.

The control data is sent over a serial connection to an Arduino Pro Micro, which computes individual motor commands and commands a dual motor controller.

## Development Notes

### Prerequisites
To run this project, you must have a ROS Noetic installation setup on a Raspberry Pi. This repository should be checked out into a Catkin workspace. For more information, see the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials).

You must also have the following dependencies installed: 
- Python 3 (`sudo apt install python3.8 && sudo apt install python3-pip`)
- OpenCV (`pip install opencv-python`)
- CvBridge (`sudo apt install ros-noetic-cv-bridge`)
- PySerial (`pip install pyserial`)
- rosserial_python (`sudo apt install ros-noetic-rosserial-python`)
- robot_upstart (`sudo apt install ros-noetic-robot-upstart`)
- video_stream_opencv (`sudo apt install ros-noetic-video-stream-opencv`)
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

### Topics

- `/camera/raspicam_node/image`: An image from the Raspberry Pi camera.
- `/line_follower/processed_image`: The processed image: threshold, reverse perspective transform.
- `/command/speed`: A commanded speed for the Arduino to achieve.
- `/command/direction`: A commanded direction for the Arduino to achieve.