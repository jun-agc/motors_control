#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

class Twist2int64():
    def __init__(self):
        self.command_left = Int64()
        self.command_right = Int64()
        self.received_twist = None
        rospy.init_node('Twist2int64')
        rospy.Subscriber('/cmd_vel', Twist, self.callback)

        
        self.pub_motor1 = rospy.Publisher('motor1/cmd_vel', Int64, queue_size=10)#name, topic_type, size
        self.pub_motor2 = rospy.Publisher('motor2/cmd_vel', Int64, queue_size=10)#name, topic_type, size
        self.pub_motor3 = rospy.Publisher('motor3/cmd_vel', Int64, queue_size=10)#name, topic_type, size

    def main_twist2int64(self):

        rospy.spin()

    def callback(self, message):
        self.received_twist = message #input data
        self.command_motor1, self.command_motor2, self.command_motor3 = self.twist2rpm(self.received_twist)
        self.pub_motor1.publish(self.command_motor1)
        self.pub_motor2.publish(self.command_motor2)
        self.pub_motor3.publish(self.command_motor3)
#        if 0.0 < self.received_twist.linear.x < 0.4:
#                rate = self.received_twist.linear.y/self.received_twist.linear.x
#                if rate >= 1:
#                        self.received_twist.linear.x = 0.4
#                        self.received_twist.linear.y = rate*0.4
#                if rate < 1:
#                        self.received_twist.linear.x = 0.4/rate
#                        self.received_twist.linear.y = 0.4
#                if rate == 0:
#                        break
        
                
        vx_send = str(self.received_twist.linear.x)
        vy_send = str(self.received_twist.linear.y)
        oz_send = str(self.received_twist.angular.z)
        send_str = vx_send + ',' + vy_send + ',' + oz_send + '\n'
        ascii_values = []
        for character in send_str:
                ascii_values.append(ord(character))
        #print(ascii_values)
        #send_str = ord(send_str)
        #send_list = send_str.split(',')
        #send_bynary = bytes(send_list)
        print(send_str)
        print("open_port")
        ser = serial.Serial('/dev/serial0', 115200)
        #print(ser.name)
        #ser.write(send_str)
        ser.write(ascii_values)
        #print("finish")
        ser.close()

    def twist2rpm(self, received_data):#convert to speed
        #(m/s, rad/s)
        wheeles_size = 0.05#wheel size
        axle_length = 0.20#axle_size(2d)

        vx = received_data.linear.x#(m/s)
        vy = received_data.linear.y#(m/s)
        vz = received_data.linear.z#(m/s)
        omegaz = received_data.angular.z#(rad/s)

        v1 = vx + axle_length*omegaz
        v2 = (-0.5)*vx + 0.866*vy + axle_length*omegaz
        v3 = (-0.5)*vx - 0.866*vy + axle_length*omegaz
        
        v1 = v1/(wheeles_size * 2 * math.pi) #wheel_speed(1/s)
        v2 = v2/(wheeles_size * 2 * math.pi) #wheel_speed(1/s)
        v3 = v3/(wheeles_size * 2 * math.pi) #wheel_speed(1/s)
        
        rpm_motor1 = 60 * v1 * 104 #gear rate
        rpm_motor2 = 60 * v2 * 104 #gear rate
        rpm_motor3 = 60 * v3 * 104 #gear rate

        return rpm_motor1, rpm_motor2, rpm_motor3

#Main Program
Convert = Twist2int64()
Convert.main_twist2int64()

#pub memo
    #rostopic pub motor/twist/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

