#!/usr/bin/env python
#
# Sonar GUI monitor (ROS-based)
#

import sys
import signal
import roslib; roslib.load_manifest('Echoes')
import rospy
from PyQt4.Qt import *
from Echoes.msg import Sonar
from Echoes.srv import Led

class MessageListenerThread(QThread):
	def run(self):
		rospy.spin()

class LedManager():
	def __init__(self):
		self.green_status = False

	def toggleGreen(self):
		if self.green_status == False:
			self.green_status = True
		else: self.green_status = False
		# TODO timeout
		rospy.wait_for_service('led_data')
		led_data = rospy.ServiceProxy('led_data', Led)
		return led_data(editGreen = True, editYellow = False, editRed = False, 
					greenIsOn = self.green_status)
	
	def changeRed(self, n):
		# TODO timeout
		rospy.wait_for_service('led_data')
		led_data = rospy.ServiceProxy('led_data', Led)
		return led_data(editGreen = False,
						editYellow = False,
						editRed = True,
						redNumOn = n)

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
        
        self.green_btn = QPushButton('Comanda led',self.widget)
        self.red_btn = QLabel("Numero led rossi: ", self.widget)
        self.red_btn_number = QSlider(Qt.Horizontal, self.widget)
        self.red_btn_number.setMinimum(0);
        self.red_btn_number.setMaximum(4);
        self.led = LedManager()
        
        # bind widgets to layout    
        layout.addWidget(QLabel('North:', self.widget), 1, 1, 1, 2)
        layout.addWidget(self.north_d, 1, 3, 1, 3)
        layout.addWidget(QLabel('South:', self.widget), 2, 1, 1, 2)
        layout.addWidget(self.south_d, 2, 3, 1, 3)
        layout.addWidget(QLabel('East:', self.widget), 3, 1, 1, 2)
        layout.addWidget(self.east_d, 3, 3, 1, 3)
        layout.addWidget(QLabel('West:', self.widget), 4, 1, 1, 2)
        layout.addWidget(self.west_d, 4, 3, 1, 3)
    	layout.addWidget(self.green_btn, 5, 1, 1, 1)
    	layout.addWidget(self.red_btn, 5, 2, 1, 1)
    	layout.addWidget(self.red_btn_number, 5, 3, 1, 1)
    	
    	self.widget.connect(self.green_btn, SIGNAL("clicked()"), self.led.toggleGreen)
    	self.widget.connect(self.red_btn_number, SIGNAL("sliderReleased()"), self.changeRedLed)
    
    def changeRedLed(self):
    	print "Chiamato change red led"
    	self.led.changeRed(self.red_btn_number.value())
    
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
    # Catch SIGINT
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    thr = MessageListenerThread()
    thr.start()
    # GUI start
    gui.start()
