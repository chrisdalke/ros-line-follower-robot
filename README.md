# ROS Line Following Robot

![Line Follower Robot](/assets/images/robot-2.JPEG)

This project is a small line-following robot. It can detect and follow tape lines on my floor! 
The robot runs ROS on a Raspberry Pi, uses OpenCV to detect lines, and an Arduino Pro Micro to control
differential steering.

I built this small project to sharpen my ROS and OpenCV skills. Your mileage may vary; the OpenCV code
likely will not work in a different lighting environment. The perception system won't generalize since it relies on the line being significantly _lighter_ than the floor, which works in my workspace.

## The 

Todo...

## Dependencies

- ROS
- ROSSerial Python
- OpenCV 2

## Development Workflow

### Starting/Stopping the Background Service

The entire stack can be launched in the background using `ros_upstart`.

To start the background service:

To stop the background service: