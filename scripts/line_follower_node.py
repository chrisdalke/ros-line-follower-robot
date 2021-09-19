import rospy
import sys

class LineFollower:
    def __init__(self):
        print("Initializing line follower node")
        # read rate config
        self.rate = rospy.Rate(rospy.get_param("/rate/lineFollower")) 

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('line_follower')
    line_follower = LineFollower()
    line_follower.run()