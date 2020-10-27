#!/usr/bin/env python


# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        # Format for drone_command
        self.cmd_drone = edrone_cmd()
        self.cmd_drone.rcRoll = 1500
        self.cmd_drone.rcPitch = 1500
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 0

        # The altitude of the drone
        self.altitude = 0

        # Desired height
        self.set_altitude = 5

        # Initial settings for the values of Kp, Ki and Kd
        self.Kp = 50
        self.Ki = 0.32
        self.Kd = 90
        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.error = 0
        self.prev_error = 0
        self.error_sum = 0

        self.out_altitude = 0
        self.min_value = 1000
        self.max_value = 2000
        #
        
        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        # ------------------------------------------------------------------------------------------------------------

    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data

    # Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
    # but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
    # so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
    # We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
    # Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
    # rosmsg show sensor_msgs/Imu

    def gps_callback(self, msg):

        self.altitude = msg.altitude


        

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):

        # Calculating the error
        self.error = self.set_altitude - self.altitude

        self.error_sum = self.error_sum + self.error

        # Calculating pid values
        self.out_altitude = (self.Kp * self.error) + (self.Ki * self.error_sum) + ((self.Kd * (self.error - self.prev_error))/self.sample_time)

        # Changing the previous sum value
        self.prev_error = self.error

        # Giving drone command
        self.cmd_drone.rcThrottle = 1500 + self.out_altitude

        if self.cmd_drone.rcThrottle > self.max_value:
            self.cmd_drone.rcThrottle = self.max_value
        elif self.cmd_drone.rcThrottle < self.min_value:
            self.cmd_drone.rcThrottle = self.min_value
        else:
            self.cmd_drone.rcThrottle = self.cmd_drone.rcThrottle

        #
        #
        #
        # ------------------------------------------------------------------------------------------------------------------------

        self.cmd_pub.publish(self.cmd_drone)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
