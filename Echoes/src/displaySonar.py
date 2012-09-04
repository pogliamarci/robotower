#!/usr/bin/env python
#
# A simple GUI to display sonar values as sent by Echoes through ROS
#

import sys
import signal
import roslib; roslib.load_manifest('Echoes')
import rospy
from PyQt4.Qt import *
from Echoes.msg import Sonar

class MessageListenerThread(QThread):
    def run(self):
        rospy.spin()

class SonarMonitorGui():
    def __init__(self): 
        # general stuff
        self.app = QApplication(sys.argv)
        self.widget = QWidget()
        self.widget.setWindowTitle('SpyKee Sonar Monitor')
        self.widget.resize(250, 350)
        layout = QGridLayout(self.widget)
        self.widget.setLayout(layout)

        # let's change the font
        font = self.app.font()
        font.setPointSize(font.pointSize()*1.5)
        self.app.setFont(font)

        # widget declaration
        self.north_d = QLCDNumber(self.widget)
        self.south_d = QLCDNumber(self.widget)
        self.east_d = QLCDNumber(self.widget)
        self.west_d = QLCDNumber(self.widget)
        
        # bind widgets to layout
        layout.addWidget(QLabel('North:', self.widget), 1, 1, 1, 1)
        layout.addWidget(self.north_d, 1, 3, 1, 2)
        layout.addWidget(QLabel('South:', self.widget), 2, 1, 1, 1)
        layout.addWidget(self.south_d, 2, 3, 1, 2)
        layout.addWidget(QLabel('East:', self.widget), 3, 1, 1, 1)
        layout.addWidget(self.east_d, 3, 3, 1, 2)
        layout.addWidget(QLabel('West:', self.widget), 4, 1, 1, 1)
        layout.addWidget(self.west_d, 4, 3, 1, 2)

    def start(self):
        self.widget.show()
        sys.exit(self.app.exec_())
        
    def sonarDataCallback(self, sonar_data):
        self.north_d.display(sonar_data.north)
        self.south_d.display(sonar_data.south)
        self.east_d.display(sonar_data.east)
        self.west_d.display(sonar_data.west)

if __name__ == "__main__":
    # ROS initialization
    rospy.init_node('sonar_monitor', anonymous=True)
    
    # Catch SIGINT (otherwise app won't exit on Ctrl+C)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # GUI initialization
    gui = SonarMonitorGui()
    rospy.Subscriber("sonar_data", Sonar, gui.sonarDataCallback)
    
    thr = MessageListenerThread()
    thr.start()
    gui.start()
