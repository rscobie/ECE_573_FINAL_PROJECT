#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import numpy as np

#from PyQt5 import QtWidgets
from PyQt5.QtWidgets import * #QApplication, QMainWindow, QPushButton, QVBoxLayout
import sys

class MyWindow(QMainWindow):
    def __init__(self):
        print("Entering mywindow\n")
        rospy.init_node('gui', anonymous=True)
        self.rate = rospy.Rate(5)
        self.s_num = Float32MultiArray()
        self.s_num.data = []
        self.pub = rospy.Publisher('/chunk_coordinate', Float32MultiArray, queue_size=10)

        super(MyWindow, self).__init__()
        #self.setGeometry(500, 500, 700, 700)
        self.setStyleSheet('background-color:orange')
        self.setWindowTitle("Dynamic World Generator")
        self.b2_flag = 0
        self.initUI()

        
    
    def publishFunct(self):
        #publishing stuffs
        #rospy.init_node('gui', anonymous=True)
        #poisson_pub()
        
        #rospy.spin()
        #self.publishFunct()
        #self.s_num.data.clear()
        while not rospy.is_shutdown():
            self.s_num.data.append(self.latValue)
            self.s_num.data.append(self.lonValue)
            #self.s_num.data.append(self.graphicVal)
            for i in self.s_num.data:
                print("an element ",i, "\n")
            rospy.loginfo(self.s_num)
            self.pub.publish(self.s_num)
            self.rate.sleep()
            break
            

    def clicked(self):
        #self.latBox.delete()
        #self.lonBox.delete()
        #self.setGeometry(500, 500, 600, 600)
        #self.publishFunct()

        self.latValue = self.latBox.text()
        self.lonValue = self.lonBox.text()
        self.graphicVal = self.drop_down.currentText()

        print("lat and long and message ", 
        self.latValue, " ", self.lonValue, " ", self.graphicVal, "\n")

        #self.labelClick.setText("Current Location is below")
        #self.labelClick.adjustSize()
        #self.labelClick.setGeometry(250, 250, 300, 25)


        #delete curent button
        self.bl.deleteLater()
        self.label2.deleteLater()
        self.drop_down.deleteLater()
        self.label.deleteLater()

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
        #self.grid = QGridLayout()

        if(self.b2_flag):
            self.b2.deleteLater()
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

        self.drop_down = QComboBox(self)
        item_list = ["ok", "good", "excelent"]
        self.drop_down.setStyleSheet('background-color:white')
        self.drop_down.addItems(item_list)
        self.drop_down.setGeometry(250, 200, 125, 30)
        self.drop_down.show()

        #set message lable
        self.label = QLabel(self)
        self.label.setText("Please Enter Latitude and Longitude: ")
        self.label.adjustSize()
        self.label.setGeometry(250, 250, 300, 25)

        #create latitude input box
        self.latBox = QLineEdit(self)
        self.latBox.setPlaceholderText("Enter Longitude")
        self.latBox.setStyleSheet('background-color:white')
        self.latBox.setGeometry(250, 300, 125, 30)

        #create longitude input box
        self.lonBox = QLineEdit(self)
        self.lonBox.setPlaceholderText("Enter Latitude")
        self.lonBox.setStyleSheet('background-color:white')
        self.lonBox.setGeometry(400, 300, 125, 30)
        
        #create start button
        self.bl = QPushButton(self)
        self.bl.setText("Start")
        self.bl.setStyleSheet("background-color:gray")
        self.bl.clicked.connect(self.clicked)
        self.bl.setGeometry(250, 350, 100, 50)
        self.bl.show()
        

        #grid.addWidget(label, 1, 0)
        #grid.addWidget(latBox, 1, 1)
        #grid.addWidget(lonBox, 2, 0)
        #grid.addWidget(bl, 2, 1)


def window():
    
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        window()
        #rospy.init_node('gui', anonymous=True)
        #poisson_pub()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
    