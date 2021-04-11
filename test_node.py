#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import trimesh
import subprocess

class test_node:
    def __init__(self):
        self.listSize = 0
        self.rate = rospy.Rate(5)
        self.coord_sub = rospy.Subscriber("/chunk_coordinate", Float32MultiArray, self.coord_callback)
        self.path_sub = rospy.Subscriber("/chunk_path", String, self.path_callback)
        self.coord_pub = rospy.Publisher('/chunk_coordinate', Float32MultiArray, queue_size=10)
        self.coordinate = list()
        self.path = ""
        self.path_cb_fired = False
        self.coord_cb_fired = False

    def path_callback(self, data):
        self.path = data.data
        self.path_cb_fired = True
        self.rate.sleep()

    def coord_callback(self, data):
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
        self.coordinate = data.data
        self.coord_cb_fired = True
        self.rate.sleep()

    """
    Test whether user can input coordinates, number of chunks, speed, etc
    """
    def test_B1(self):
        print("testing B1...")
        print("starting gui...")
        subprocess.Popen("rosrun ECE_573_FINAL_PROJECT gui_node.py", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("please enter 43.7 for latitude, and 7.42 for longitude into gui then press start, this cannot be done automatically")
        while not self.coord_cb_fired: #wait for user to enter coordinates
            rospy.sleep(1)
        self.coord_cb_fired = False
        if round(self.coordinate[0],5) != 43.7 or round(self.coordinate[1],5) != 7.42:
            print("failed")
            return
        print("passed")

    """
    Test whether chunk is 1000x1000m
    """
    def test_B2(self):
        print("testing B2...")
        subprocess.Popen("rosrun ECE_573_FINAL_PROJECT osm_to_building.py", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        pub_val = Float32MultiArray()
        pub_val.data = [32.2319,-110.9501]
        while self.coord_pub.get_num_connections() < 2:
            rospy.sleep(1)
        self.coord_pub.publish(pub_val)
        while not self.path_cb_fired: #wait for generation_node to finish
            rospy.sleep(1)
        self.path_cb_fired = False
        model = trimesh.load_mesh(self.path)
        transform, extents = trimesh.bounds.oriented_bounds(model)
        if extents[1] < 1 or extents[2] < 1: #generation node uses 1km as scaling unit, so 1 means 1km
            print("failed")
            return
        print("passed")

if __name__ == '__main__':
    subprocess.Popen('roscore', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    rospy.sleep(5)
    rospy.init_node('test_node', anonymous=True)
    tester = test_node()
    tester.test_B1()
    tester.test_B2()
    rospy.spin()
