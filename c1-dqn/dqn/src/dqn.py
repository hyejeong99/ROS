#!/usr/bin/env python
import rospy, rospkg, os, time
import numpy as np
from model import *
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor

def ultrasonic_callback(data):
    global angle, speed, motor_pub, motor_msg
    receive = data.data 
    val = 2.76
    
    state = np.array([min(int(receive[0]*val), 276), min(int(receive[1]*val),276), min(int(receive[2]*val),276), min(int(receive[3]*val),276), min(int(receive[4]*val),276), angle])
    action = (study_get_action(state) - 1)

    if action == -1 :
        angle = -20 #left
    elif action == 0 :
        angle = 0 #right
    else :
        angle = 20

    #angle += int(action)
    #angle = min(30, max(-30, angle))
    motor_msg.speed = speed
    motor_msg.angle = int(round(angle * 1.67))

    motor_pub.publish(motor_msg)

if __name__ == '__main__':
    basic_speed = 14
    basic_angle = 0

    angle = basic_angle
    speed = basic_speed

    hidden_layer = [256, 256]

    study_init(6, hidden_layer, 0.001, "DQN")
    view_epi = 203
    dqn_package_path = rospkg.RosPack().get_path('dqn')
    os.chdir(dqn_package_path)
    
    study_model_load(view_epi)
    #time.sleep(0.5)

    rospy.init_node("dqn")
    motor_msg = xycar_motor()
    rospy.Subscriber("/xycar_ultrasonic", Int32MultiArray, ultrasonic_callback, queue_size=1)
    motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    rospy.spin()
