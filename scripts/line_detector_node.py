import rospy
import sys
import cv2
import traceback
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

class LineDetector:
    def __init__(self):
        print("Initializing line detector node")
        # read rate config
        self.rate = rospy.Rate(rospy.get_param("/rate/lineDetector")) 
        self.speed = rospy.get_param("speed")
        self.steer_multiplier = rospy.get_param("steering_multiplier")
        self.image_sub_rpi = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.image_callback_compressed)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback_raw)
        self.image_pub = rospy.Publisher("processed_image", Image)
        self.speed_pub = rospy.Publisher('/motor_driver/speed', Float32)
        self.dir_pub = rospy.Publisher('/motor_driver/direction', Float32)
        self.bridge = CvBridge()
        self.line_offset = 0

    def image_callback_raw(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.process_image(cv_image)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            rospy.logerr("CvBridge Error, skipped image frame!")

    def image_callback_compressed(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_image(image_np)
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr("skipped processed image frame!")

    def process_image(self, cv_image):
        # Downscale to 256x256
        cv_image = cv2.resize(cv_image, (256, 256), interpolation = cv2.INTER_AREA)

        # Change to grayscale and blur a bit
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.GaussianBlur(cv_image, (7, 7), 0)

        # Threshold the image
        (T, threshold_image) = cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY)

        # Overlay black box on top of image
        threshold_image = cv2.rectangle(threshold_image, (0, 200), (256, 200), (0, 0, 0), -1)

        # Get center of the thresholded image
        M = cv2.moments(threshold_image)

        # calculate x,y coordinate of center
        cX = 128
        cY = 128
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

        # Compute an offset in [-1, 1] coordinates to convert to steering
        self.line_offset = (cX - 128.0) / 128.0

        # Change from grayscale
        threshold_image = cv2.cvtColor(threshold_image, cv2.COLOR_GRAY2RGB)

        # Draw some debugging info
        
        #Center of bottom section
        threshold_image = cv2.circle(threshold_image, (cX, cY), 5, (0, 255, 0), -1)

        # The centerline of the image
        threshold_image = cv2.line(threshold_image,(128,0),(128,256),(0,0,255),1)

        # Bottom quadrant that is used to signal which direction to go
        threshold_image = cv2.line(threshold_image,(0,200),(256,200),(0,0,255),1)

        # visualize line offset
        threshold_image = cv2.line(threshold_image,(int(128 + (self.line_offset * 128)),0),(int(128 + (self.line_offset * 128)),256),(255,0,255),1)

        # Output the processed message
        image_message = self.bridge.cv2_to_imgmsg(threshold_image, "passthrough")
        self.image_pub.publish(image_message)

        # Output the control speed / direction
        self.dir_target = self.line_offset * self.steer_multiplier
        self.dir_slowdown = 1.0 - (abs(self.dir_target) * 0.75)
        self.speed_pub.publish(self.speed * self.dir_slowdown)
        self.dir_pub.publish(self.dir_target)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('line_detector')
    line_detector = LineDetector()
    line_detector.run()
