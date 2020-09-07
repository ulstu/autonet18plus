#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import sys

class Camera:
    is_started = False

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        self.pub_bottom = rospy.Publisher('cam_bottom', Image, queue_size=1)
        self.pub_top = rospy.Publisher('cam_top', Image, queue_size=1)
        self.bridge = CvBridge()
        self.cap_bottom = cv2.VideoCapture(rospy.get_param("~cam_bottom_id"))
        self.cap_top = cv2.VideoCapture(rospy.get_param("~cam_top_id"))
        rospy.Subscriber("cmd", String, self.callback)

        self.run()
        rospy.spin()
        
    def get_image(self, id):
        if id == 'top':
            _, frame = self.cap_top.read()
        elif id == 'bottom':
            _, frame = self.cap_bottom.read()
        return frame

    def resize_image(self, name):
        img = self.get_image(name)
        scale_percent = int(rospy.get_param("~image_scale"))
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
        #rospy.loginfo("img {} resized to {} {}".format(name, width, height))
        return self.bridge.cv2_to_imgmsg(resized, "bgr8")


    def run(self):
        while not rospy.is_shutdown():
            if self.is_started:
                try:
                   self.pub_top.publish(self.resize_image('top'))
                   self.pub_bottom.publish(self.resize_image('bottom'))
                except:
                    rospy.loginfo("Error {}".format(sys.exc_info()))                
                #except CvBridgeError as e:
                #    rospy.loginfo(e)
            rospy.sleep(1 / int(rospy.get_param("~fps")))
    
    def callback(self, data):
        data = data.data
        if 'START' in data:
            self.is_started = True
        elif 'STOP' in data or 'INTERRUPT' in data:
            self.is_started = False
        

if __name__ == '__main__':
    try:
        Camera()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка cam node: {}'.format(traceback.format_exc()))
