#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool  # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import numpy as np
import tf

class Kalman:
    def __init__(self, tao):
        # Inicializar las matrices del filtro de Kalman
        self.tao = tao
        self.A = np.eye(3)  # Matriz de transición de estados
        self.B = np.zeros((3, 2))  # Matriz de control
        self.H = np.eye(3)  # Matriz de observación
        self.Q = np.eye(3)  #* 0.1  # Covarianza del ruido de proceso
        self.R = np.eye(3)  * 0.05  # Covarianza del ruido de medida  0.05 
        self.P = np.eye(3)  # Matriz de covarianza del error

    def predict(self, x, u):
        # Predicción del estado y de la covarianza del error
        self.B = np.array([
            [self.tao * np.cos(x[2]), 0],
            [self.tao * np.sin(x[2]), 0],
            [0, self.tao]
        ])
        x_pred = self.A @ x + self.B @ u
        P_pred = self.A @ self.P @ self.A.T + self.Q
        return x_pred, P_pred

    def update(self, x_pred, P_pred, z):
        # Actualización del estado y de la covarianza del error
        y = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        x_upd = x_pred + K @ y
        P_upd = (np.eye(3) - K @ self.H) @ P_pred
        return x_upd, P_upd

    def kalmanCalculation(self, x, u, z):
        # Cálculo del filtro de Kalman
        x_pred, P_pred = self.predict(x, u)
        x_upd, P_upd = self.update(x_pred, P_pred, z)
        self.P = P_upd
        return x_upd

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower', anonymous=True)
        rospy.loginfo("Path Follower Initialized")

        self.velocity_command = Twist()
        self.rate = rospy.Rate(10)

        self.optimal_path = None
        self.opt_path_receive = False
        self.pathFollower_activationKey = False
        self.actual_pose = np.array([0.0, 0.0, 0.0])

        # Kalman Filter
        self.tao = 0.1
        self.kalman = Kalman(self.tao)
        self.x = np.array([0.0, 0.0, 0.0])  # Estado inicial

        # Publishers and Subscribers
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/rrt_path', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/seguidor_key', Bool, self.activationKey)
        rospy.wait_for_message("/odom", Odometry, timeout=30)

    def activationKey(self, msg):
        self.pathFollower_activationKey = msg.data

    def path_callback(self, data):
        for pose_stamped in data.poses:
            pose_stamped.pose.position.x = -pose_stamped.pose.position.x
            pose_stamped.pose.position.y = -pose_stamped.pose.position.y
        self.optimal_path = data
        if not self.opt_path_receive and self.pathFollower_activationKey:
            self.follow_optimal_path()
            self.opt_path_receive = True

    def odom_callback(self, data):
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.actual_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y, yaw])

    def follow_optimal_path(self):
        threshold_linear_error = 0.1
        threshold_rotational_error = 0.35  # 0.4

        if self.optimal_path is None:
            rospy.logwarn("Optimal path data is missing.")
            return

        for pose_stamped in self.optimal_path.poses:
            xd = pose_stamped.pose.position.x
            yd = pose_stamped.pose.position.y
            thetad = self.estimate_theta(xd, yd)
            desired_pose = np.array([xd, yd, thetad])

            while True:
                error = self.calculate_error(xd, yd, thetad)
                # print(error)
                if abs(error[1]) >= threshold_rotational_error:
                    self.calculate_rotational_velocity(error)
                    self.velocity_pub.publish(self.velocity_command)
                    self.rate.sleep()
                elif abs(error[0]) >= threshold_linear_error:
                    self.calculate_linear_velocity(error)
                    self.velocity_pub.publish(self.velocity_command)
                    self.rate.sleep()
                else:
                    break

                # Update state estimate with Kalman filter
                u = np.array([self.velocity_command.linear.x, self.velocity_command.angular.z])
                z = self.actual_pose
                self.x = self.kalman.kalmanCalculation(self.x, u, z)

            self.velocity_pub.publish(Twist())

        self.velocity_pub.publish(Twist())

    def estimate_theta(self, xd, yd):
        y = yd - self.actual_pose[1]
        x = xd - self.actual_pose[0]
        thetad = np.arctan2(y, x)
        return thetad

    def calculate_error(self, xd, yd, thetad):
        error_x = xd - self.actual_pose[0]
        error_y = yd - self.actual_pose[1]
        error_dist = np.sqrt(error_x**2 + error_y**2)

        error_theta = thetad - self.actual_pose[2]
        if error_theta > np.pi:
            error_theta -= 2 * np.pi
        elif error_theta < -np.pi:
            error_theta += 2 * np.pi

        return np.array([error_dist, error_theta])

    def calculate_linear_velocity(self, error):
        v_max_linear = 0.1
        self.velocity_command.linear.x = v_max_linear * np.tanh(4.0 * error[0])

    def calculate_rotational_velocity(self, error):
        v_max_angular = 0.27
        self.velocity_command.angular.z = np.clip(0.8 * error[1], -v_max_angular, v_max_angular)

if __name__ == '__main__':
    path_follower = PathFollower()
    rospy.spin()
