#!/usr/bin/env python

import rospy
import threading
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped


rospy.init_node("change_l", anonymous=True)
pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=1)

rate = rospy.Rate(100)
hello_str = AckermannDriveStamped()

start_t=time.time()


while not rospy.is_shutdown():
    print (time.time()-start_t)
    if(time.time()-start_t) <= 2.0 :
        hello_str.drive.steering_angle=0.0
        hello_str.drive.speed=3.0
        pub.publish(hello_str)
    elif (time.time()-start_t) > 2.0 :
        if(time.time()-start_t) < 3.2 :
            hello_str.drive.steering_angle=0.05
            hello_str.drive.speed=3.0
            pub.publish(hello_str)
        elif(time.time()-start_t) < 4.8 :
            hello_str.drive.steering_angle= -0.07
            hello_str.drive.speed=3.0
            pub.publish(hello_str)
    	else :
    	    hello_str.drive.steering_angle=0.0
    	    hello_str.drive.speed=3.0
    	    pub.publish(hello_str)

