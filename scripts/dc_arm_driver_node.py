#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray as IntArray
#from sensor_msgs.msg import Joy
#from geometry_msgs.msg import Twist

#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

#Constants
nbPCAServo=16 

#Parameters
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]


class dc_arm_driver_node: 
    
    def __init__(self):
        self.pca = ServoKit(channels=16)
        self.driver_init()
        rospy.init_node('dc_arm_driver', anonymous=True)
        rospy.Subscriber("arm_cmd", IntArray, self.callback)
        rospy.spin()

    def callback(self, msg):
        if msg.data[0] == -1:
            self.release_all()
        else:
            for i in range(1, len(msg.data)):
                self.pca.servo[i-1].angle = msg.data[i]
    
    def driver_init(self):
        for i in range(nbPCAServo):
            self.pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])

    def release_all(self):
        for i in range(nbPCAServo):
            self.pca.servo[i].angle=None #disable channel


if __name__ == '__main__':
    try:
        dc_arm_driver_node()
    except rospy.ROSInterruptException:
        pass