#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

class poisson_sub:
    def __init__(self):
        self.listSize = 0
        self.rate = rospy.Rate(5)
        self.sub = rospy.Subscriber("/chunk_coordinate", Float32MultiArray, self.callback)

    def callback(self, data):
        #testing published values
        print("Entering callback")
        #testing for an empty list
        if(len(data.data) == 0):
            raise Exception("Error, one of more coordinate values are missing\n")
        #testing for a list containing empty elements
        else:
            for items in data.data:
                print ("value is ", items)
                if(items == ''):
                    raise Exception("Error: the published values have a problem")

        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('minumum', anonymous=True)
    poisson_sub()
    rospy.spin()
