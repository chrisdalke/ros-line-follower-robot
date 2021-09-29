import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from enum import Enum
import math
import serial
import sys

class MotorDriver:
    def __init__(self, port):
        print("Initializing motor driver & comms node")
        self.rate = rospy.Rate(rospy.get_param("/rate/motorDriver"))
        self.timeout = rospy.get_param("/motor_driver/timeout")
        self.port_name = port
        self.open_port()
        self.speed_command = 0.0
        self.dir_command = 0.0
        self.has_command = True
        self.timed_out = False
        self.last_command_time = rospy.get_rostime()
        self.speed_sub = rospy.Subscriber('speed', Float32, self.speed_callback)
        self.dir_sub = rospy.Subscriber('direction', Float32, self.dir_callback)

    def open_port(self):
        try:
            self.port = serial.Serial(self.port_name)
            self.port_open = True
        except:
            rospy.logerr('Unable to open port:' + self.port1_name) 
            self.port_open = False

    def speed_callback(self, msg):
        self.speed_command = msg.data
        self.last_command_time = rospy.get_rostime()
        self.has_command = True
        self.timed_out = False
        rospy.logdebug('Received SPEED_COMMAND = ' + str(self.speed_command))

    def dir_callback(self, msg):
        self.dir_command = msg.data
        self.last_command_time = rospy.get_rostime()
        self.has_command = True
        self.timed_out = False
        rospy.logdebug('Received DIR_COMMAND = ' + str(self.dir_command))
    
    def send_command(self):
        if self.port_open:
            self.port.write('wss.speed=' + str(self.speed_command))
            self.port.write('wss.dir=' + str(self.dir_command))
            rospy.logdebug('Sending SPEED_COMMAND = ' + str(self.speed_command))
            rospy.logdebug('Sending DIR_COMMAND = ' + str(self.dir_command))

    def run(self):
        while not rospy.is_shutdown():
            # If the serial port is not open, attempt to reconnect
            if not self.port_open:
                self.open_port()

            # Check timeout 
            if (rospy.get_rostime() - self.last_command_time).to_sec() > self.command_timeout:
                if not self.timed_out:
                    rospy.logdebug('No command received, stopping robot.')
                    self.speed_command = 0
                    self.dir_command = 0
                    self.has_command = True
                    self.timed_out = True
            
            if self.has_command:
                self.send_command()
                self.has_command = False

            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('motor_driver')
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) >= 1:
        line_follower = LineFollower(sys.argv[1])
        line_follower.run()
    else:
        print("Invalid number of args")