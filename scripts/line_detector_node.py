import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class LineDetector:
    def __init__(self):
        print("Initializing line detector node")
        # read rate config
        self.rate = rospy.Rate(rospy.get_param("/rate/lineDetector")) 
        self.image_sub_rpi = rospy.Subscriber("/camera/raspicam_node/image", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("processed_image", Image)
        self.speed_pub = rospy.Publisher('/motor_driver/speed', Float32)
        self.dir_pub = rospy.Publisher('/motor_driver/direction', Float32)
        self.bridge = CvBridge()
        self.line_offset = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.process_image(cv_image)
        except:
            rospy.logerr("CvBridge Error, skipped image frame!")

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
        self.speed_pub.publish(1.0)
        self.dir_pub.publish(self.line_offset)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('line_detector')
    line_detector = LineDetector()
    line_detector.run()
