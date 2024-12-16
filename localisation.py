#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32
from tf.transformations import quaternion_from_euler
import numpy as np
import time

class DifferentialRobot:
    def __init__(self):
        rospy.init_node('differential_robot')
        self.rate = rospy.Rate(10)  # 10 Hz
        self.wheel_base = 0.1738  # Distancia entre las llantas (en metros)
        self.wheel_radius = 0.0509 
        self.wl_robot = 0.0
        self.wr_robot = 0.0
        self.prev = 0
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        rospy.Subscriber('/wl', Float32, self.wl_callback)
        rospy.Subscriber('/wr', Float32, self.wr_callback)

    def wl_callback(self, msg):
        self.wl_robot = msg.data

    def wr_callback(self, msg):
        self.wr_robot = msg.data
    
    def calculate_odometry(self):
        while not rospy.is_shutdown():
            # Verificar si se ha solicitado el cierre del nodo de ROS
            if rospy.is_shutdown():
                break
            
            # Calcular la velocidad angular de cada llanta

            linear_vel = self.wheel_radius * (self.wr_robot + self.wl_robot) / 2
            angular_vel = self.wheel_radius * (self.wr_robot - self.wl_robot) / self.wheel_base
            
            # Actualizar la pose del robot
            dt = time.time() - self.prev
            self.pose['x'] += linear_vel * np.cos(self.pose['theta']) * dt
            self.pose['y'] += linear_vel * np.sin(self.pose['theta']) * dt
            self.pose['theta'] += angular_vel * dt

            if (self.pose['theta'] > np.pi):
                self.pose['theta'] -= 2*np.pi
            elif (self.pose['theta'] < -np.pi):
                self.pose['theta'] += 2*np.pi
            
            # Publicar la odometria
            q = quaternion_from_euler(0, 0, self.pose['theta'])
            quaternion = Quaternion(q[0], q[1], q[2], q[3])
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.pose['x']
            odom_msg.pose.pose.position.y = self.pose['y']
            odom_msg.pose.pose.orientation = quaternion
            odom_msg.twist.twist.linear.x = linear_vel
            odom_msg.twist.twist.angular.z = angular_vel
            self.odom_pub.publish(odom_msg)
        

            self.prev = time.time()  # Tiempo transcurrido entre iteraciones
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = DifferentialRobot()
        robot.calculate_odometry()
    except rospy.ROSInterruptException:
        pass
