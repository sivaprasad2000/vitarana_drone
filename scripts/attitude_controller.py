#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf

value_of_pi = 3.141592653589

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        self.drone_orientation_euler = [0.0, 0.0, 0.0]


        self.setpoint_cmd = [1500.0, 1500.0, 1500.0, 1500.0]

        self.setpoint_euler = [0.0, 0.0, 0.0]

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.Kp = [4.2, 4.2, 400]
        self.Ki = [0.2, 0.2, 0]
        self.Kd = [4.5, 4.5, 0]

        self.throttle = 0
        self.prev_values = [0,0,0]
        self.error = [0,0,0]
        self.error_sum = [0,0,0]
        self.min_values = [0,0,0,0]
        self.max_values = [1024,1024,1024,1024]
        self.out_roll = 0
        self.out_pitch = 0
        self.out_yaw = 0

        self.sample_time = 0.060  # in seconds

        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)

    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w


    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle

    def pid(self):
 
        # Converting quaternion to euler angles
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -100 degree to 100 degree for axes
        self.setpoint_euler[0] = (self.setpoint_cmd[0] * 0.2) - 300
        self.setpoint_euler[1] = (self.setpoint_cmd[1] * 0.2) - 300
        self.setpoint_euler[2] = (self.setpoint_cmd[2] * 0.2) - 300

        # Complete the equations for pitch and yaw axis

        # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itself
        self.throttle = (self.setpoint_cmd[3] * 1.024) - 1024

        # Calculating the error
        self.error[0] = self.setpoint_euler[0] - (self.drone_orientation_euler[0]*(180/value_of_pi))
        self.error[1] = self.setpoint_euler[1] - (self.drone_orientation_euler[1]*(180/value_of_pi))
        self.error[2] = self.setpoint_euler[2] - (self.drone_orientation_euler[2]*(180/value_of_pi))


        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        self.error_sum[2] = self.error_sum[2] + self.error[2]

        # Calculating pid values
        self.out_roll = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.error_sum[0]) + ((self.Kd[0] * (self.error[0] - self.prev_values[0]))/self.sample_time)
        self.out_pitch = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.error_sum[1]) + ((self.Kd[1] * (self.error[1] - self.prev_values[1]))/self.sample_time)
        self.out_yaw = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.error_sum[2]) + ((self.Kd[2] * (self.error[2] - self.prev_values[2]))/self.sample_time)

        # Changing the previous sum value
        self.prev_values[0] = self.error[0]
        self.prev_values[1] = self.error[1]
        self.prev_values[2] = self.error[2]

        # Giving pwm values
        self.pwm_cmd.prop1 = self.throttle - self.out_roll + self.out_pitch - self.out_yaw
        self.pwm_cmd.prop2 = self.throttle - self.out_roll - self.out_pitch + self.out_yaw
        self.pwm_cmd.prop3 = self.throttle + self.out_roll - self.out_pitch - self.out_yaw
        self.pwm_cmd.prop4 = self.throttle + self.out_roll + self.out_pitch + self.out_yaw

        if self.pwm_cmd.prop1 > self.max_values[0]:
            self.pwm_cmd.prop1 = self.max_values[0]
        elif self.pwm_cmd.prop1 < self.min_values[0]:
            self.pwm_cmd.prop1 = self.min_values[0]
        else:
            self.pwm_cmd.prop1 = self.pwm_cmd.prop1

        if self.pwm_cmd.prop2 > self.max_values[1]:
            self.pwm_cmd.prop2 = self.max_values[1]
        elif self.pwm_cmd.prop2 < self.min_values[1]:
            self.pwm_cmd.prop2 = self.min_values[1]
        else:
            self.pwm_cmd.prop2 = self.pwm_cmd.prop2
        
        if self.pwm_cmd.prop3 > self.max_values[2]:
            self.pwm_cmd.prop3 = self.max_values[2]
        elif self.pwm_cmd.prop3 < self.min_values[2]:
            self.pwm_cmd.prop3 = self.min_values[2]
        else:
            self.pwm_cmd.prop3 = self.pwm_cmd.prop3

        if self.pwm_cmd.prop4 > self.max_values[3]:
            self.pwm_cmd.prop4 = self.max_values[3]
        elif self.pwm_cmd.prop4 < self.min_values[3]:
            self.pwm_cmd.prop4 = self.min_values[3]
        else:
            self.pwm_cmd.prop4 = self.pwm_cmd.prop4


        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
