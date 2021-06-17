#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
import numpy as np
from xycar_msgs.msg import xycar_sensor
from sensor_msgs.msg import CompressedImage, Imu, Image
#from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


print('start')

#객체 생성
#compress_camera_msg = CompressedImage()
camera_msg = Image()
#imu_msg = Imu()
#us_msg = Int32MultiArray()

#노드가 CompressedImage타입의 메세지를 사용한 토픽을 pub
#pub_compress_camera = rospy.Publisher("/usb_cam/image_raw/compressed", CompressedImage, queue_size=1)
pub_camera = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
#노드가 Int32MultiArray타입의 메세지를 사용한 토픽을 pub
#pub_us = rospy.Publisher("/xycar_ultrasonic", Int32MultiArray, queue_size=1)
#노드가 Imu타입의 메세지를 사용한 토픽을 pub
#pub_imu = rospy.Publisher("/imu", Imu, queue_size=1)

bridge = CvBridge()

def callback(msg):
    global bridge
    global camera_msg#, imu_msg, us_msg, compress_camera_msg
    global pub_camera#, pub_imu, pub_us,pub_compress_camera
    #print(msg.ultra_sonic)
    
    #if msg.camera_b:#camera_b에 데이터가 있다면
    print("받았니??")
    '''
    compress_camera_msg = msg.camera
    #노드가 CompressedImage타입의 메세지를 사용한 토픽을 pub
    pub_compress_camera.publish(compress_camera_msg)

    #cmpressed 안된 이미지
    np_arr = np.fromstring(compress_camera_msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    print(len(cv_image))
    #camera_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    #print(camera_msg)
    '''
    if msg.camera_b:
        pub_camera.publish(msg.camera)


    '''
    #cmpressed 안된 이미지
    np_arr = np.fromstring(compress_camera_msg.data, np.uint8)
    #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv_image = cv2.imdecode(np_arr, cv2.COLOR_BGR2GRAY)
    print(len(cv_image))
    camera_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    #print(camera_msg)

    pub_camera.publish(camera_msg)
    '''
    
    
    '''
    if msg.imu_b:#imu_b에 데이터가 있다면
        imu_msg = msg.imu
        #노드가 Imu타입의 메세지를 사용한 토픽을 pub
        pub_imu.publish(imu_msg)
    


    if msg.ultra_sonic_b:#ultra_sonic_b에 메세지가 있다면
        us_msg = msg.ultra_sonic
        #노드가 Int32MultiArray타입의 메세지를 사용한 토픽을 pub
        pub_us.publish(us_msg)
    '''
    
print(1)

#노드의 rospy 이름을 'classify_sensor'로 초기화
rospy.init_node("classify_sensor", anonymous=True)
#xycar_sensor_info라는 토픽명으로 발행된 토픽을 구독
rospy.Subscriber("xycar_sensor_info", xycar_sensor, callback, queue_size=1)
#토픽을 받을 때까지 기다린다
rospy.spin()
