#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


#from PyQt5 import QtWidgets
from PyQt5.QtWidgets import * #QApplication, QMainWindow, QPushButton, QVBoxLayout
import sys

class MyWindow(QMainWindow):
    def __init__(self):
        print("Entering mywindow\n")
        rospy.init_node('gui', anonymous=True)
        self.incVal_Flag = 0
        self.rate = rospy.Rate(50)
        self.s_num = Float32MultiArray()
        self.s_num.data = []
        self.pub = rospy.Publisher('/chunk_coordinate', Float32MultiArray, queue_size=10)
        #self.sub = rospy.Subscriber("/prius_diff_drive_controller/odom", Odometry, self.callback)
        self.sub2 = rospy.Subscriber("/prius_diff_drive_controller/cmd_vel", Twist, self.callback2)


        super(MyWindow, self).__init__()
        #self.setGeometry(500, 500, 700, 700)
        self.setStyleSheet('background-color:orange')
        self.setWindowTitle("Dynamic World Generator")
        self.b2_flag = 0
        self.initUI()
    
    def publishFunct(self):
        while not rospy.is_shutdown():
            self.s_num.data.append(float(self.latValue))
            self.s_num.data.append(float(self.lonValue))
            #self.s_num.data.append(self.graphicVal)
            for i in self.s_num.data:
                print("an element ",i, "\n")
            rospy.loginfo(self.s_num)
            self.pub.publish(self.s_num)
            self.s_num.data = list()
            break
            self.rate.sleep()
  
    #def callback(self, data):
        #print("Entering callback data: ", data)
     #   count  = 1
      #  for i in data:
       #     print("line ", count, ": ", i)
        #for i in  data.data:
         #   print(i)
         
    def callback2(self, data):
        #print("Entering callback data: ", data)
        #print("velosity is ", data.linear.x)
        print(self.start_flag)
        if(self.start_flag):
            self.vel_Box.setText(str(data.linear.x))
            self.latBox.setText(str(data.angular.x))
            self.lonBox.setText(str(data.angular.y))

           
    def clicked(self):
        self.start_flag = 1
        try:
            self.latValue = self.latBox.text()
            self.lonValue = self.lonBox.text()
            self.graphicVal = self.drop_down.currentText()
        except ValueError:
            self.latValue = 0
            self.lonValue = 0
            self.incVal_Flag = 1
            print("Incorrects values entered ")
            self.inc_label = QLabel(self)
            self.inc_label.setText("Wrong values please try again \n")
            self.inc_label.setGeometry(250, 250, 400, 400)

        #delete curent button
        self.bl.deleteLater()
        self.label2.deleteLater()
        self.drop_down.deleteLater()
        self.label.deleteLater()
        if(self.incVal_Flag):
            self.inc_label.deleteLater()
            incVal_Flag = 0
        
        #velocity label
        self.vel_label = QLabel(self)
        self.vel_label.setText("Current Velocity ")
        self.vel_label.setGeometry(250, 150, 300, 25)
        self.vel_label.show()
        
        #velocity box
        self.vel_Box = QLineEdit(self)
        self.vel_Box.setPlaceholderText("Velocity")
        self.vel_Box.setStyleSheet('background-color:white')
        self.vel_Box.setGeometry(250, 200, 125, 30)
        self.vel_Box.show()

        #lantidude Label
        self.lat_label = QLabel(self)
        self.lat_label.setText("Latitude")
        self.lat_label.setGeometry(250, 250, 125, 25)
        self.lat_label.show()

        #longitude label
        self.lon_label = QLabel(self)
        self.lon_label.setText("Longitude")
        self.lon_label.setGeometry(400, 250, 125, 25)
        self.lon_label.show()

        self.b2 = QPushButton(self)
        self.b2.setText("Stop")
        self.b2.setStyleSheet("background-color:red")
        self.b2.clicked.connect(self.initUI)
        self.b2.setGeometry(250, 350, 100, 50)
        self.b2.show()
        self.b2_flag = 1
        self.publishFunct()

    def initUI(self):
        self.setGeometry(500, 500, 700, 700)
        self.start_flag = 0
        #self.grid = QGridLayout()

        if(self.b2_flag):
            self.b2.deleteLater()
            self.vel_Box.deleteLater()
            self.vel_label.deleteLater()
            self.lat_label.deleteLater()
            self.lon_label.deleteLater()
            self.lat_label.deleteLater()
            self.b2_flag = 0

        #set message lable
        self.label2 = QLabel(self)
        self.label2.setText("Enter Graphic Quality: ")
        self.label2.setGeometry(250, 150, 300, 25)
        self.label2.show()

        #create input box
        #self.qBox = QLineEdit(self)
        #self.qBox.setPlaceholderText("Enter Quality")
        #self.qBox.setStyleSheet('background-color:white')
        #self.qBox.setGeometry(250, 200, 125, 30)

        #quality drop down option
        self.drop_down = QComboBox(self)
        item_list = ["ok", "good", "excellent"]
        self.drop_down.setStyleSheet('background-color:white')
        self.drop_down.addItems(item_list)
        self.drop_down.setGeometry(250, 200, 125, 30)
        self.drop_down.show()

        #set message lable
        self.label = QLabel(self)
        self.label.setText("Please Enter Latitude and Longitude: ")
        self.label.adjustSize()
        self.label.setGeometry(250, 250, 300, 25)
        self.label.show()

        #create latitude input box
        self.latBox = QLineEdit(self)
        self.latBox.setPlaceholderText("Enter Latitude")
        self.latBox.setStyleSheet('background-color:white')
        self.latBox.setGeometry(250, 300, 125, 30)
        self.latBox.show()

        #create longitude input box
        self.lonBox = QLineEdit(self)
        self.lonBox.setPlaceholderText("Enter Longitude")
        self.lonBox.setStyleSheet('background-color:white')
        self.lonBox.setGeometry(400, 300, 125, 30)
        self.lonBox.show()
        
        #create start button
        self.bl = QPushButton(self)
        self.bl.setText("Start")
        self.bl.setStyleSheet("background-color:gray")
        self.bl.clicked.connect(self.clicked)
        self.bl.setGeometry(250, 350, 100, 50)
        self.bl.show()
        
def window():
    
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        window()
    except rospy.ROSInterruptException:
        pass
    
