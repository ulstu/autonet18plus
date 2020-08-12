#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import String
import sys

class Button:
    green_pin = 0
    red_pin = 0
    is_active = {}

    def __init__(self):
        rospy.init_node('button_node', anonymous=True)

        self.green_pin = int(rospy.get_param("~green_button_pin"))
        self.red_pin = int(rospy.get_param("~red_button_pin"))
        self.is_active = {self.green_pin: True, self.red_pin: True}

        self.pub = rospy.Publisher('cmd', String, queue_size=10)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.green_pin, GPIO.IN)
        GPIO.setup(self.red_pin, GPIO.IN)

        self.run()
        rospy.spin()

    def check_button(self, pin, name, start_cmd, stop_cmd, delay):
        if not GPIO.input(pin):
            if self.is_active[pin]:
                rospy.loginfo('{} button pressed. Delay before publishing cmd: {} seconds'.format(name, delay))
                rospy.sleep(delay)
                rospy.loginfo('{} command invoked'.format(name, start_cmd))
                self.pub.publish(start_cmd)
                self.is_active[pin] = False
        else:
            if not self.is_active[pin]:
                rospy.loginfo('{} button resumed. {} command invoked'.format(name, stop_cmd))
                self.pub.publish(stop_cmd)
                self.is_active[pin] = True

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.check_button(self.green_pin, 'green', 'START', 'STOP', int(rospy.get_param("~start_delay")))
                self.check_button(self.red_pin, 'red', 'INIT', 'INTERRUPT', 0)
                rospy.sleep(0.1)
            except:
                rospy.error("Error {}".format(sys.exc_info()[0]))
                rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        Button()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка button node: {}'.format(traceback.format_exc()))
