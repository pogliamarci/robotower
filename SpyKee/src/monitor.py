#!/usr/bin/env python
#
# Sonar GUI monitor (ROS-based)
#

import sys
import roslib; roslib.load_manifest('SpyKee')
import rospy
from PyQt4.Qt import *
from SpyKee.msg import Motion

class SonarMonitorGui():
    def __init__(self): 
        self.app = QApplication(sys.argv)
        self.widget = QWidget()
        self.widget.setWindowTitle('SpyKee GUI Control')
        self.widget.resize(300, 200)
        layout = QGridLayout(self.widget)
        self.widget.setLayout(layout)
        
        self.pub = rospy.Publisher('spykee_motion', Motion)
        
        self.left_track_box = QLineEdit(self.widget)
        self.right_track_box = QLineEdit(self.widget)
        self.commit_btn = QPushButton(self.widget)
        
        # bind widgets to layout    
        layout.addWidget(QLabel('Left:', self.widget), 1, 1, 1, 2)
        layout.addWidget(self.left_track_box, 1, 3, 1, 3)
        layout.addWidget(QLabel('Right:', self.widget), 2, 1, 1, 2)
        layout.addWidget(self.right_track_box, 2, 3, 1, 3)
        layout.addWidget(self.commit_btn, 3, 1, 1, 5)
    	
    	self.widget.connect(self.commit_btn, SIGNAL("clicked()"), self.commitMotionValues)
    
    def start(self):
        self.widget.show()
        sys.exit(self.app.exec_())
        
    def commitMotionValues(self):
        left_track = self.left_track_box.text().toInt()[0]
        right_track = self.right_track_box.text().toInt()[0]
        self.pub.publish(leftTrack = left_track, rightTrack = right_track)
        
        
if __name__ == "__main__":  
    # GUI initialization
    gui = SonarMonitorGui()
    # ROS initialization
    rospy.init_node('spykeeMonitor', anonymous=True)
    # GUI start
    gui.start()
