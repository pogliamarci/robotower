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
    def run(self):
        print 'thread started'
        rospy.spin()

class ImageWidget(QWidget):
    
    new_frame_sig = pyqtSignal(QImage)
    
    def __init__(self, width, heigth, parent = None):
        super(ImageWidget, self).__init__(parent)
        self._frame = QImage("frame.jpg")
        self._width = width
        self._heigth = heigth
        self.setMinimumSize(self._width, self._heigth)
        self.setMaximumSize(self._width, self._heigth)
        self.new_frame_sig.connect(self._onNewFrame)
    
    def _onNewFrame(self, image):
        self._frame = image
        self.update()
    
    def paintEvent(self, e):
        if self._frame is None:
            return
        painter = QPainter(self)
        painter.drawImage(QPoint(0,0), self._frame)

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
        
        self.left_track_box = QLineEdit(self.widget)
        self.right_track_box = QLineEdit(self.widget)
        self.commit_btn = QPushButton(self.widget)

        self.camera_widget = ImageWidget(320, 240, self.widget)

        # bind widgets to layout
        layout.addWidget(QLabel('Left:', self.widget), 1, 1, 1, 2)
        layout.addWidget(self.left_track_box, 1, 3, 1, 3)
        layout.addWidget(QLabel('Right:', self.widget), 2, 1, 1, 2)
        layout.addWidget(self.right_track_box, 2, 3, 1, 3)
        layout.addWidget(self.commit_btn, 3, 1, 1, 5)
    	layout.addWidget(self.camera_widget, 4, 1, 1, 5)
        
        self.widget.connect(self.commit_btn, SIGNAL("clicked()"), self.commitMotionValues)
    
    def start(self):
        self.widget.show()
        sys.exit(self.app.exec_())
        
    def commitMotionValues(self):
        left_track = self.left_track_box.text().toInt()[0]
        right_track = self.right_track_box.text().toInt()[0]
        self.pub.publish(leftTrack = left_track, rightTrack = right_track)

class ImageSubscriber():
    def __init__(self, widget_obj, filename = "frane.jpg"):
        self._widget_obj = widget_obj
        self._filename = filename
    
    def callback(self, img):
        myfile = open(self._filename, "w")
        myfile.write(img.data)
        myfile.close
        self._widget_obj.new_frame_sig.emit(QImage(self._filename))

if __name__ == "__main__":  
    # GUI initialization
    gui = SonarMonitorGui()
    thr = MessageListenerThread()
    im = ImageSubscriber(gui.camera_widget)
    # ROS initialization
    rospy.init_node('spykeeMonitor', anonymous=True)
    rospy.Subscriber("spykee_camera", CompressedImage, im.callback)
    
    thr.start()
    gui.start()
