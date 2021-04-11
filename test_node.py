#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

class poisson_sub:
    def __init__(self):
        self.listSize = 0
        self.min = Float32()
        self.rate = rospy.Rate(5)
        #this subcriver is used for test 4
        self.sub = rospy.Subscriber("/chunk_coordinate", Float32MultiArray, self.callback)

    #call back function handles the data recieved by the publisher node on the topic random_generator    
    def callback(self, data):
        if(len(data.data) == 0):
            raise Exception("Error, one of more coordinate values are missing\n")
        #rospy.loginfo(self.min)
        #self.pub2.publish(self.min)
        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('minumum', anonymous=True)
    poisson_sub()
    rospy.spin()
