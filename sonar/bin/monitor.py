#!/usr/bin/env python
#
# Sonar GUI monitor (ROS-based)
#

import sys
import roslib; roslib.load_manifest('sonar')
import rospy
from PyQt4.Qt import *
from sonar.msg import Sonar
from sonar.srv import Led

class MessageListenerThread(QThread):
	def run(self):
		rospy.spin()

class SonarMonitorGui():
    def __init__(self): 
        self.app = QApplication(sys.argv)
        self.widget = QWidget()
        self.widget.setWindowTitle('Sonar GUI monitor')
        self.widget.resize(300, 200)
        layout = QGridLayout(self.widget)
        self.widget.setLayout(layout)
        self.north_d = QLCDNumber(self.widget)
        self.south_d = QLCDNumber(self.widget)
        self.east_d = QLCDNumber(self.widget)
        self.west_d = QLCDNumber(self.widget)
        
        # bind widgets to layout    
        layout.addWidget(QLabel('North:', self.widget), 1, 1, 1, 2)
        layout.addWidget(self.north_d, 1, 3, 1, 3)
        layout.addWidget(QLabel('South:', self.widget), 2, 1, 1, 2)
        layout.addWidget(self.south_d, 2, 3, 1, 3)
        layout.addWidget(QLabel('East:', self.widget), 3, 1, 1, 2)
        layout.addWidget(self.east_d, 3, 3, 1, 3)
        layout.addWidget(QLabel('West:', self.widget), 4, 1, 1, 2)
        layout.addWidget(self.west_d, 4, 3, 1, 3)
    
    def start(self):
        self.widget.show()
        sys.exit(self.app.exec_())
        
    def sonarDataCallback(self, sonar_data):
        self.north_d.display(sonar_data.north)
        self.south_d.display(sonar_data.south)
        self.east_d.display(sonar_data.east)
        self.west_d.display(sonar_data.west)
        
if __name__ == "__main__":  
    # GUI initialization
    gui = SonarMonitorGui()
    # ROS initialization
    rospy.init_node('sonarMonitor', anonymous=True)
    rospy.Subscriber("sonar_data", Sonar, gui.sonarDataCallback)
    thr = MessageListenerThread()
    thr.start()
    # GUI start
    gui.start()