import rospy
import sys

class LineFollower:
    def __init__(self):
        print("Initializing line follower node")
        self.rate = rospy.Rate(100) 

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('line_follower')
    myargv = rospy.myargv(argv=sys.argv)
    line_follower = LineFollower()
    line_follower.run()