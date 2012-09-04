#!/usr/bin/env python
#
# Sonar GUI monitor (ROS-based)
#

import sys, os
import math
import roslib; roslib.load_manifest('SpyKee')
import rospy
import signal
from PyQt4.Qt import *
from SpyKee.msg import Motion
from sensor_msgs.msg import CompressedImage

mutex = QMutex()

class MessageListenerThread(QThread):  
    def run(self):
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
        mutex.lock()
        painter.drawImage(QPoint(0,0), self._frame)
        mutex.unlock()

class SonarMonitorGui():
    def __init__(self): 
        self.i = 0
        self.app = QApplication(sys.argv)
        self.widget = QWidget()
        self.widget.setWindowTitle('SpyKee GUI Controller')
        self.widget.resize(300, 400)
        layout = QGridLayout(self.widget)
        self.widget.setLayout(layout)
        
        self.pub = rospy.Publisher('spykee_motion', Motion)
        
        self.tan_speed_box = QSlider(Qt.Horizontal, self.widget)
        self.tan_speed_view = QLCDNumber(self.widget)
        self.rot_speed_box = QSlider(Qt.Horizontal, self.widget)
        self.rot_speed_view = QLCDNumber(self.widget)
        self.tan_speed_box.setMinimum(-100)
        self.tan_speed_box.setMaximum(100)
        self.tan_speed_box.setValue(0)
        self.rot_speed_box.setMinimum(-100)
        self.rot_speed_box.setMaximum(100)
        self.rot_speed_box.setValue(0)
        
        self.commit_btn = QPushButton('Send commands to SpyKee!',self.widget)

        self.camera_widget = ImageWidget(320, 240, self.widget)

        # bind widgets to layout
        layout.addWidget(QLabel('Tangential speed:', self.widget), 1, 1, 1, 1)
        layout.addWidget(self.tan_speed_box, 1, 2, 1, 3)
        layout.addWidget(self.tan_speed_view, 1, 5, 1, 1)
        layout.addWidget(QLabel('Rotational speed:', self.widget), 2, 1, 1, 1)
        layout.addWidget(self.rot_speed_box, 2, 2, 1, 3)
        layout.addWidget(self.rot_speed_view, 2, 5, 1, 1)
        layout.addWidget(self.commit_btn, 3, 1, 1, 5)
    	layout.addWidget(self.camera_widget, 4, 1, 1, 5)
        
        self.widget.connect(self.commit_btn, SIGNAL("clicked()"), self.commitMotionValues)
        self.widget.connect(self.tan_speed_box, SIGNAL("valueChanged(int)"), self.changeLcdValues)
        self.widget.connect(self.rot_speed_box, SIGNAL("valueChanged(int)"), self.changeLcdValues)
    
    def changeLcdValues(self):
        self.tan_speed_view.display(self.computeSpeed(self.tan_speed_box.value()))
        self.rot_speed_view.display(self.computeSpeed(self.rot_speed_box.value()))
    
    def start(self):
        self.widget.show()
        sys.exit(self.app.exec_())
        
    def computeSpeed(self, speed):
        ''' this is just to have greater (manual) control at low speeds
        and a greater area for speed = 0 in the slider... '''
        comp = int( (speed * 9 / 100.0)**2 * 90 / 81 )
        if speed >= 0: return comp
        else: return -comp
    
    def commitMotionValues(self):
        tan_speed = self.computeSpeed( self.tan_speed_box.value() )
        rot_speed = self.computeSpeed( self.rot_speed_box.value() )
        self.pub.publish(tanSpeed = tan_speed, rotSpeed = rot_speed)

class ImageSubscriber():
    def __init__(self, widget_obj, filename = "frame.jpg"):
        self._widget_obj = widget_obj
        self._filename = filename
    
    def callback(self, img):
        mutex.lock()
        myfile = open(self._filename, "w")
        myfile.write(img.data)
        myfile.close()
        mutex.unlock()
        self._widget_obj.new_frame_sig.emit(QImage(self._filename))

if __name__ == "__main__":  
    # GUI initialization
    gui = SonarMonitorGui()
    thr = MessageListenerThread()
    im = ImageSubscriber(gui.camera_widget)
    # ROS initialization
    rospy.init_node('spykeeMonitor', anonymous=True)
    rospy.Subscriber("spykee_camera", CompressedImage, im.callback, queue_size=1, buff_size=10000)
    # catch SIGINT
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    thr.start()
    gui.start()
