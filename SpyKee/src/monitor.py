#!/usr/bin/env python
#
# Sonar GUI monitor (ROS-based)
#

import sys
import roslib; roslib.load_manifest('SpyKee')
import rospy
from PyQt4.Qt import *
from SpyKee.msg import Motion
from sensor_msgs.msg import CompressedImage


class MessageListenerThread(QThread):
    signal = pyqtSignal()
    
    def run(self):
        print 'thread started'
        rospy.spin()

thr = MessageListenerThread()

class SonarMonitorGui():
    def __init__(self): 
        self.i = 0
        self.app = QApplication(sys.argv)
        self.widget = QWidget()
        self.widget.setWindowTitle('SpyKee GUI Control')
        self.widget.resize(600, 400)
        layout = QGridLayout(self.widget)
        self.widget.setLayout(layout)
        
        self.pub = rospy.Publisher('spykee_motion', Motion)
        
        self.pix = QPixmap("myfile")
        
        self.left_track_box = QLineEdit(self.widget)
        self.right_track_box = QLineEdit(self.widget)
        self.commit_btn = QPushButton(self.widget)
    
        self.image_view = QGraphicsView(self.widget)
        self.image_view.setGeometry(0, 300, 320, 240)
        self.image_scene = QGraphicsScene()
        self.image_scene.addPixmap(self.pix)
        self.image_view.setScene(self.image_scene)
        
        # bind widgets to layout
        layout.addWidget(QLabel('Left:', self.widget), 1, 1, 1, 2)
        layout.addWidget(self.left_track_box, 1, 3, 1, 3)
        layout.addWidget(QLabel('Right:', self.widget), 2, 1, 1, 2)
        layout.addWidget(self.right_track_box, 2, 3, 1, 3)
        layout.addWidget(self.commit_btn, 3, 1, 1, 5)
    	layout.addWidget(self.image_view, 4, 1, 1, 5)
        
        self.widget.connect(self.commit_btn, SIGNAL("clicked()"), self.commitMotionValues)
        thr.signal.connect(self.commit_image_change)
    
    def start(self):
        self.widget.show()
        sys.exit(self.app.exec_())
        
    def commitMotionValues(self):
        left_track = self.left_track_box.text().toInt()[0]
        right_track = self.right_track_box.text().toInt()[0]
        self.pub.publish(leftTrack = left_track, rightTrack = right_track)
        
    def imageCallback(self, img):
        print 'sto per salvare su file', self.i
        myfile = open("myfile", "w")
        myfile.write(img.data)
        myfile.close
        self.i += 1
        thr.signal.emit()
        
    def commit_image_change(self):
        self.pix = QPixmap("myfile")
        self.image_scene = QGraphicsScene()
        self.image_scene.addPixmap(self.pix)
        self.image_view.setScene(self.image_scene)

if __name__ == "__main__":  
    # GUI initialization
    gui = SonarMonitorGui()
    # ROS initialization
    rospy.init_node('spykeeMonitor', anonymous=True)
    rospy.Subscriber("spykee_camera", CompressedImage, gui.imageCallback)
    
    thr.start()
    # GUI start
    gui.start()
