#! /usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion


class Robot():
    def __init__(self):
        self.x, self.y, self.theta, self.laser = 0, 0, 0, 0
        self.angular_speed = 90 * np.pi/180
        self.linear_speed = 0.02
        rospy.init_node("vector_node", anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odometry)
        rospy.Subscriber('/vector/laser', Range, self.laser_reader, queue_size = 1)
        self.pub = rospy.Publisher('/vector/cmd_vel', Twist, queue_size = 1)
        rospy.sleep(0.5)
        self.vel = Twist()


    def laser_reader(self, msg):
        self.laser = msg.range


    def odometry(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    

    def rotation(self, delta_theta):
        
        self.vel.angular.z = (1 if delta_theta > 0 else -1) * self.angular_speed
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        
        t = abs(delta_theta)/self.angular_speed * 2.018
        t_0 = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - t_0 < t):
            self.pub.publish(self.vel)

        self.vel.angular.z = 0
        self.pub.publish(self.vel)
       

    def translation(self, delta_trans):
        
        self.vel.angular.z = 0
        self.vel.linear.x = self.linear_speed
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        
        t = delta_trans/(self.linear_speed)
        t_0 = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - t_0 < t):
            self.pub.publish(self.vel)

        self.vel.linear.x = 0
        self.pub.publish(self.vel)


if __name__ == '__main__':
    robot = Robot()
    robot.rotation(np.pi/2)
    rospy.sleep(0.5)
    robot.translation(0.1)