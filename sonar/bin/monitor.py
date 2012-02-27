#!/usr/bin/env python
#
# Sonar GUI monitor (ROS-based)
#

import sys
import roslib; roslib.load_manifest('sonar')
import rospy
from PyQt4.Qt import *
from sonar.msg import Sonar, Led

def sonarDataCallback(sonar_data):
    north_d.display(sonar_data.north)
    south_d.display(sonar_data.south)
    east_d.display(sonar_data.east)
    west_d.display(sonar_data.west)

class MessageListenerThread(QThread):
	def run(self):
		rospy.spin()

app = QApplication(sys.argv)
widget = QWidget()
north_d = QLCDNumber(widget)
south_d = QLCDNumber(widget)
east_d = QLCDNumber(widget)
west_d = QLCDNumber(widget)
		
if __name__ == "__main__":  
    # QT initialization
    widget.resize(300, 200)
    widget.setWindowTitle('Sonar GUI monitor')
    layout = QGridLayout(widget)
    widget.setLayout(layout)

    # bind widgets to layout  
    layout.addWidget(QLabel('North:', widget), 1, 1, 1, 2)
    layout.addWidget(north_d, 1, 3, 1, 3)
    layout.addWidget(QLabel('South:', widget), 2, 1, 1, 2)
    layout.addWidget(south_d, 2, 3, 1, 3)
    layout.addWidget(QLabel('East:', widget), 3, 1, 1, 2)
    layout.addWidget(east_d, 3, 3, 1, 3)
    layout.addWidget(QLabel('West:', widget), 4, 1, 1, 2)
    layout.addWidget(west_d, 4, 3, 1, 3)

    widget.show()

    # ROS initialization
    rospy.init_node('sonarMonitor', anonymous=True)
    rospy.Subscriber("sonar_data", Sonar, sonarDataCallback)
    thr = MessageListenerThread()
    thr.start()
    # rospy.spin()
    sys.exit(app.exec_())