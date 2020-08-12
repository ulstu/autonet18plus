#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import division
from smc import SMC
import sys
from lib.Adafruit_Python_PCA9685 import PCA9685
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MotorController:

    is_dc_initialized = False
    can_move = False

    def __init__(self):
        rospy.init_node('move_node', anonymous=True)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("cmd", String, self.cmd_callback)
        self.motor_init(rospy.get_param("~motor_serial_port"))
        self.servo = PCA9685()
        self.servo.servos[0].set(signed=True, reverse=True, min=120, max=120, trim=0, exp=0)
        self.set_angle(0)
        rospy.spin()

    def set_angle(self, angle):
        angle = int(angle * 60)
        self.servo.setServo(0, min(60, max(-60, angle)))

    def motor_init(self, port):
        while not self.is_dc_initialized:
            try:
                self.mc = SMC(port, 115200)
                self.is_dc_initialized = True
            except:
                rospy.error("Error {}".format(sys.exc_info()[0]))
                rospy.sleep(0.5)

        rospy.loginfo('motor_initialized!')

    def set_speed(self, speed=0):
        speed = int(speed * 20)
        speed = min(20, max(-20, speed))
        self.mc.speed7b(speed)

    def start_motor(self):
        self.mc.init()

    def stop_motor(self):
        self.mc.stop()

    def cmd_callback(self, data):
        data = str(data)
        rospy.loginfo('motor cmd: {}'.format(data))
        if "START" in data:
            self.can_move = True
            self.start_motor()
        elif "STOP" in data or "INTERRUPT" in data:
            rospy.loginfo('motor stop')
            self.can_move = False
            self.stop_motor()

    def cmd_vel_callback(self, data):
        rospy.loginfo('motor cmd_vel: {}'.format(data))
        if self.can_move:
            self.set_speed(data.linear.x)
            self.set_angle(data.angular.z)

if __name__ == '__main__':
    try:
        MotorController()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка motor_controller node: {}'.format(traceback.format_exc()))
