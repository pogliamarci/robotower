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
from Echoes.msg import Rfid
from Echoes.msg import Towers
from Echoes.srv import Led

class MessageListenerThread(QThread):
    def run(self):
        rospy.spin()

class LedManager():
    def __init__(self):
        self.green_status = False
        self.yellowIsOn = [False, False, False, False]
    
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
        print n
        print "ok, waiting for service"
        # TODO timeout
        rospy.wait_for_service('led_data')
        led_data = rospy.ServiceProxy('led_data', Led)
        return led_data(editGreen = False, editYellow = False,
                        editRed = True, redNumOn = n)

    def toggleYellow(self, num):
        print num
        if self.yellowIsOn[num] == False:
            self.yellowIsOn[num] = True
        else:
            self.yellowIsOn[num] = False
        print self.yellowIsOn
        print "ok, waiting for service"
        # TODO timeout
        rospy.wait_for_service('led_data')
        led_data = rospy.ServiceProxy('led_data', Led)
        return led_data(editGreen = False, editYellow = True, editRed = False, yellowIsOn = self.yellowIsOn)

class SonarMonitorGui():
    def __init__(self): 
        # general stuff
        self.app = QApplication(sys.argv)
        self.widget = QWidget()
        self.widget.setWindowTitle('Sonar GUI monitor')
        self.widget.resize(300, 200)
        layout = QGridLayout(self.widget)
        self.widget.setLayout(layout)
        
        # sonar
        self.north_d = QLCDNumber(self.widget)
        self.south_d = QLCDNumber(self.widget)
        self.east_d = QLCDNumber(self.widget)
        self.west_d = QLCDNumber(self.widget)
        
        # led
        self.green_btn = QPushButton('Led verde',self.widget)
        self.red_btn_number = QSlider(Qt.Horizontal, self.widget)
        self.red_btn_number.setMinimum(0);
        self.red_btn_number.setMaximum(4);
        self.yellow_btn = []
        for i in range(4):
        	self.yellow_btn.append(QPushButton(str(i), self.widget))
        self.led = LedManager()
        
        # tower and factories
        self.towerStatus = QLabel('---')
        self.factoriesStatus = QLabel('---')
        
        # RFID
        
        # bind widgets to layout    
        # sonar
        layout.addWidget(QLabel('North:', self.widget), 1, 1, 1, 2)
        layout.addWidget(self.north_d, 1, 3, 1, 2)
        layout.addWidget(QLabel('South:', self.widget), 2, 1, 1, 2)
        layout.addWidget(self.south_d, 2, 3, 1, 2)
        layout.addWidget(QLabel('East:', self.widget), 3, 1, 1, 2)
        layout.addWidget(self.east_d, 3, 3, 1, 2)
        layout.addWidget(QLabel('West:', self.widget), 4, 1, 1, 2)
        layout.addWidget(self.west_d, 4, 3, 1, 2)
        layout.addWidget(self.green_btn, 5, 1, 1, 1)
        # led
        layout.addWidget(QLabel("Num. led rossi: ", self.widget), 5, 2, 1, 1)
        layout.addWidget(self.red_btn_number, 5, 3, 1, 2)
        layout.addWidget(QLabel('Led gialli:', self.widget), 6,1,1,1)
        for i in range(4):
            layout.addWidget(self.yellow_btn[i], 7, i+1, 1, 1)
        # towers, factories
        layout.addWidget(self.towerStatus, 8, 1, 1, 2)
        layout.addWidget(self.factoriesStatus, 8, 3, 1, 2)

        # connection btw signals and slots
        self.green_btn.clicked.connect(self.led.toggleGreen)
        self.red_btn_number.sliderReleased.connect(lambda : self.led.changeRed(self.red_btn_number.value()))
        self.yellow_btn[0].clicked.connect(lambda : self.led.toggleYellow(0))
        self.yellow_btn[1].clicked.connect(lambda : self.led.toggleYellow(1))
        self.yellow_btn[2].clicked.connect(lambda : self.led.toggleYellow(2))
        self.yellow_btn[3].clicked.connect(lambda : self.led.toggleYellow(3))   

    def start(self):
        self.widget.show()
        sys.exit(self.app.exec_())
        
    def sonarDataCallback(self, sonar_data):
        self.north_d.display(sonar_data.north)
        self.south_d.display(sonar_data.south)
        self.east_d.display(sonar_data.east)
        self.west_d.display(sonar_data.west)

    def rfidCallback(self, rfid_data):
        self.popup = QLabel("RFID: " + rfid_data.id)
        self.popup.show()

    def towerCallback(self, tower_data):
        if tower_data.isTowerDestroyed == 1:
            self.towerStatus = "Torre distrutta!!!"
        else:
            self.towerStatus = "Torre ok"
        self.factoriesStatus = "Fabbriche abbattute: " + tower_data.destroyedFactories

if __name__ == "__main__":  
    # GUI initialization
    gui = SonarMonitorGui()
    
    # ROS initialization
    rospy.init_node('sonarMonitor', anonymous=True)
    rospy.Subscriber("sonar_data", Sonar, gui.sonarDataCallback)
    rospy.Subscriber("rfid_data", Rfid, gui.rfidCallback)
    rospy.Subscriber("towers_data", Towers, gui.towerCallback)
    
    # Catch SIGINT
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    thr = MessageListenerThread()
    thr.start()
    # GUI start
    gui.start()
