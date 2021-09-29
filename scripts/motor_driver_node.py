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
        self.speed_sub = rospy.Subscriber('/motor_driver/speed', Float32, self.speed_callback)
        self.dir_sub = rospy.Subscriber('/motor_driver/direction', Float32, self.dir_callback)

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
        print('Received SPEED_COMMAND = ' + str(self.speed_command))

    def dir_callback(self, msg):
        self.dir_command = msg.data
        self.last_command_time = rospy.get_rostime()
        self.has_command = True
        self.timed_out = False
        print('Received DIR_COMMAND = ' + str(self.dir_command))
    
    def send_command(self):
        if self.port_open:
            speedStr = 'wss.speed=' + str(self.speed_command)
            dirStr = 'wss.dir=' + str(self.dir_command)
            self.port.write(speedStr.encode())
            self.port.write(dirStr.encode())
            print('Sending SPEED_COMMAND = ' + str(self.speed_command))
            print('Sending DIR_COMMAND = ' + str(self.dir_command))

    def run(self):
        while not rospy.is_shutdown():
            # If the serial port is not open, attempt to reconnect
            if not self.port_open:
                self.open_port()

            # Check timeout 
            if (rospy.get_rostime() - self.last_command_time).to_sec() > self.timeout:
                if not self.timed_out:
                    print('No command received, stopping robot.')
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
        motor_driver = MotorDriver(sys.argv[1])
        motor_driver.run()
    else:
        print("Invalid number of args")
