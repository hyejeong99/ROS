#!/usr/bin/env python

import rospy, rospkg
import cv2
import numpy as np

#from xycar_motor.msg import xycar_motor
#from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
#from sensor_msgs.msg import Imu
from xycar_msgs.msg import xycar_sensor
#from std_msgs.msg import Int32MultiArray

rospy.init_node('xycar_sensor_c1_1')

info = xycar_sensor()
info.camera_b = False
#info.imu_b = False
#info.ultra_sonic_b = False

freq = rospy.get_param('~freq')
#imu_topic = rospy.get_param('~imu_topic')
camera_topic = rospy.get_param('~camera_topic')
#sonic_topic = rospy.get_param('~sonic_topic')
car_num = rospy.get_param('~car_num')

info.carnum = car_num

#comp_image = CompressedImage()

def yolo_callback(msg):
    global info
    info.camera_b = True
    info.camera = msg

'''
def image_callback(msg):
    global info
    info.camera_b = True
    info.camera = msg
def imu_callback(msg):
    global info
    info.imu_b = True
    info.imu = msg

def sonic_callback(msg):
    global info
    info.ultra_sonic_b = True
    info.ultra_sonic = msg
'''

rospy.Subscriber("/usb_cam/image_raw", Image , yolo_callback)
#rospy.Subscriber(camera_topic, CompressedImage , image_callback)
#rospy.Subscriber(imu_topic, Imu, imu_callback, queue_size=1)
#rospy.Subscriber(sonic_topic, Int32MultiArray, sonic_callback, queue_size=1)

pub = rospy.Publisher('xycar_sensor_info', xycar_sensor, queue_size=1)
r = rospy.Rate(freq)
print("done")
while not rospy.is_shutdown():
    global info
    r.sleep()
    pub.publish(info)
    print(info.camera_b)
    #print(info.imu_b)
    #print(info.ultra_sonic_b)
    #print(info.ultra_sonic)
    #info = xycar_sensor()
    info.camera_b = False