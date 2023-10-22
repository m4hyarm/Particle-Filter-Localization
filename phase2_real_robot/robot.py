#! /usr/bin/env python3
import rospy
import numpy as np
from anki_vector_ros.msg import Drive
from rospy import Publisher
from time import sleep
from anki_vector_ros.msg import Proximity


class Robot():
    def __init__(self):
        self.laser = 0
        self.linear_speed = 50
        rospy.init_node("vector_node", anonymous=True)
        rospy.Subscriber('/proximity', Proximity, self.laser_reader)
        self.pub = Publisher("/motors/wheels", Drive, queue_size=1)
        rospy.sleep(0.5)


    def laser_reader(self, msg):
        self.laser = msg.distance
    

    def rotation(self, direction):
        
        speed = self.linear_speed * direction
        r = 176 / (2 * np.pi)
        o = abs(self.linear_speed) / r
        u = (r / (48 * 2)) * 2 * o
        duration = np.pi / (2 * u)
        
        sleep(0.5)
        self.pub.publish(-speed, speed, 0.0, 0.0)
        sleep(duration*1.04)
        self.pub.publish(0.0, 0.0, 0.0, 0.0)
       

    def translation(self, delta_trans):

        duration = delta_trans / self.linear_speed
        
        sleep(0.5)
        self.pub.publish(self.linear_speed, self.linear_speed, 0.0, 0.0)
        sleep(duration)
        self.pub.publish(0.0, 0.0, 0.0, 0.0)


if __name__ == '__main__':
    robot = Robot()
    robot.rotation(1)
    rospy.sleep(0.5)
    robot.translation(150)