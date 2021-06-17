#!/usr/bin/env python

import rospy
import threading
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

rospy.init_node("change_l", anonymous=True)
pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=1)

rate = rospy.Rate(100)
hello_str = AckermannDriveStamped()

start_t=time.time() //시작 시간

while not rospy.is_shutdown(): //끝나기 전가지
    print (time.time()-start_t) //현재 시간-시작 시간
    if(time.time()-start_t) <= 2.0 : //시작 후 2초 까지
        hello_str.drive.steering_angle=0.0 //직진
        hello_str.drive.speed=3.0 //속도
        pub.publish(hello_str)
    elif (time.time()-start_t) > 2.0 : //2초 지나면 
        if(time.time()-start_t) < 3.2 :
            hello_str.drive.steering_angle=0.05 //왼족으로
            hello_str.drive.speed=3.0
            pub.publish(hello_str)
        elif(time.time()-start_t) < 4.8 : //4.8초 지나면
            hello_str.drive.steering_angle= -0.07 //오른쪽으로
            hello_str.drive.speed=3.0
            pub.publish(hello_str)
        else : //그 이후에는
            hello_str.drive.steering_angle=0.0 //왼쪽으로
            hello_str.drive.speed=3.0
            pub.publish(hello_str)
