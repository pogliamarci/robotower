#!/usr/bin/env python
import roslib; roslib.load_manifest('sonar')
import rospy
from std_msgs.msg import String
from sonar.msg import Sonar, Led

def talker():
    pub = rospy.Publisher('sonar_data', Sonar)
    rospy.init_node('talker')
    i = 0
    while not rospy.is_shutdown():
        rospy.loginfo(str)
        pub.publish(Sonar(0.5*i, 0.3*i, 0.1*i, 0.2*i))
        rospy.sleep(1.0)
        i += 1
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass