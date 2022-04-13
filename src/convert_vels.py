#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time

#### DESCRIPTION ####
# This file provide conversion between cmd_vel command and specific rotation commands on wheels #
# User provide cmd_vel message and then using this converter this message is translated to robot and commands are published on robot's wheels #
#### DESCRIPTION ####

class ConvertVelocities(object):

    def __init__(self):
        # Create publishers for each wheel
        self.cmd_pub_l_f = rospy.Publisher('/rotrac_e2/front_left_wheel_velocity_controller/command',Float64,queue_size=1)
        self.cmd_pub_l_r = rospy.Publisher('/rotrac_e2/rear_left_wheel_velocity_controller/command',Float64,queue_size=1)
        self.cmd_pub_r_f = rospy.Publisher('/rotrac_e2/front_right_wheel_velocity_controller/command',Float64,queue_size=1)
        self.cmd_pub_r_r = rospy.Publisher('/rotrac_e2/rear_right_wheel_velocity_controller/command',Float64,queue_size=1)
        # Create subscriber for cmd_vel message
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.callback)
        
        self.Vl = Float64()         # Initialize object for handling left side velocity
        self.Vr = Float64()         # Initialize object for handling right side velocity

        self.cmd_vel = Twist()      # Initialize object for cmd_vel message
        self.rate = rospy.Rate(2)   # Set frequency
        
        self.rate.sleep()           # Wait for subscriber


    def callback(self,msg):
        ### Callback function will convert the velocity commands ###
        self.cmd_vel = msg
        self.V = self.cmd_vel.linear.x
        self.w = self.cmd_vel.angular.z
        
        L = 1.150       #distance between front and rear wheel
        W = 0.820       #distance between L and R wheel
        R = 0.246       #wheel's radius

        self.Vl = (2*float(self.V) - float(self.w)*W) / (2*R)       # Calculate the right wheels velocity command                                        
        self.Vr = (2*float(self.V) + float(self.w)*W) / (2*R)       # Calculate the left wheels velocity command         


    def pub_velocities(self):
        ### Publish commands to robot's wheels ###
        rospy.loginfo('Start publishing velocity commands...')
        while not rospy.is_shutdown():
            self.cmd_pub_l_f.publish(self.Vl)
            self.cmd_pub_l_r.publish(self.Vl)
            self.cmd_pub_r_f.publish(self.Vr)
            self.cmd_pub_r_r.publish(self.Vr)

            self.rate.sleep()       

if __name__ == '__main__':
    rospy.init_node('convert_velocities_node')
    CV = ConvertVelocities()
    CV.pub_velocities()


