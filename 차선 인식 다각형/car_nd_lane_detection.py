#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, copy, time
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from pid import PID

pub = None

bridge = CvBridge()
cv_image = np.empty(shape=[0])

Width = 640
Height = 480

pid=PID(0.55, 0, 0.4)

last_left_fit = None
last_right_fit = None
left_fit = None
right_fit = None

window_title = 'camera'

warp_img_w = 300
warp_img_h = 300

warpx_margin = 10
warpx_margin_m = 5
warpy_margin = 4
warpy_margin_m = 2

nwindows = 9
margin = 12
minpix = 5

'''
warp_src = np.float32([
    [270-warpx_margin_m-65, 262-warpy_margin_m],#50-150-200
    [355+warpx_margin_m+15, 262+warpy_margin_m],#15-50-100
    [490+warpx_margin+65, 440+warpy_margin],
    [150-warpx_margin-15, 440-warpy_margin]
])
warp_dist = np.float32([
    [0,0],
    [warp_img_w,0],
    [warp_img_w, warp_img_h],
    [0,warp_img_h]
])
'''

calibrated = True
if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

#indic warp point
def point(warpx_margin_m, warpx_margin):
    global warp_img_w
    global warp_img_h

    warp_src = np.float32([
        [270-warpx_margin_m-65, 262-warpy_margin_m],#50-150-200
        [355+warpx_margin_m+15, 262-warpy_margin_m],
        #[355+warpx_margin_m+15, 262+warpy_margin_m],#15-50-100
        [490+warpx_margin+65, 440+warpy_margin],
        [150-warpx_margin-15, 440+warpy_margin]
        #[150-warpx_margin-15, 440-warpy_margin]
    ])
    warp_dist = np.float32([
        [0,0],
        [warp_img_w,0],
        [warp_img_w, warp_img_h],
        [0,warp_img_h]
    ])
    return warp_src, warp_dist

def img_callback(data):
    global cv_image    
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

def calibrate_image(frame):#roi image
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))

def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def warp_process_image(img, last_left_fit, last_right_fit):
    global nwindows
    global margin
    global minpix
    global left_fit, right_fit

    #color filter
    blur = cv2.GaussianBlur(img,(5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)) #채널 분리, hlv로
    _, lane = cv2.threshold(L, 130, 255, cv2.THRESH_BINARY) #135-130, 130이하, 255이상 버리기
    cv2.imshow("L", lane) 
    #return
    histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0)
    out_img = np.dstack((lane, lane, lane))*255

    midpoint = np.int(histogram.shape[0]/2)
    leftx_current = np.argmax(histogram[:midpoint])
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint

    window_height = np.int(lane.shape[0]/nwindows)
    nz = lane.nonzero()

    left_lane_inds = []
    right_lane_inds = []
    for window in range(nwindows):
        win_yl = lane.shape[0] - (window+1)*window_height
        win_yh = lane.shape[0] - window*window_height

        win_xll = leftx_current - margin
        win_xlh = leftx_current + margin
        win_xrl = rightx_current - margin
        win_xrh = rightx_current + margin

        #draw line
        cv2.line(out_img,(leftx_current,win_yl),(leftx_current,win_yh),(120,120,0), 3) 
        cv2.line(out_img,(rightx_current,win_yl),(rightx_current,win_yh),(120,120,0), 3) 

        #lane interval
        laneInt=win_xrl-win_xlh
        if(laneInt<0):
            rospy.logerr("Line Overlap")
            laneInt=abs(laneInt)
        print("Lane Interval : "+str(laneInt))

        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 

        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nz[1][good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    #detect empty lane
    if(len(left_lane_inds)<1): 
        if(len(right_lane_inds)<1):
            left_fit = last_left_fit
            right_fit = last_right_fit
        elif(len(right_lane_inds)!=0):
            left_fit = last_left_fit
            right_fit = np.polyfit(nz[0][right_lane_inds] , nz[1][right_lane_inds], 2)
            if((win_xrl-win_xlh)<150):
                out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
        rospy.logerr("Line Missing")
    elif(len(right_lane_inds)<1): 
        if(len(left_lane_inds)<1):
            left_fit = last_left_fit
            right_fit = last_right_fit
        elif(len(left_lane_inds)!=0):
            left_fit = np.polyfit(nz[0][left_lane_inds], nz[1][left_lane_inds], 2)
            right_fit = last_right_fit
            if((win_xrl-win_xlh)<150):
                out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
        rospy.logerr("Line Missing")
    else:
        left_fit = np.polyfit(nz[0][left_lane_inds], nz[1][left_lane_inds], 2)
        right_fit = np.polyfit(nz[0][right_lane_inds] , nz[1][right_lane_inds], 2)
        if((win_xrl-win_xlh)<150):
            out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
        else:
            out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
            out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]

    #out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    #out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
    cv2.imshow("viewer", out_img)

    #return left_fit, right_fit
    return left_fit, right_fit, leftx_current, rightx_current  

def draw_lane(image, warp_img, Minv, left_fit, right_fit):
    global Width, Height
    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    center_fitx = (left_fitx + right_fitx) / 2

    #plus
    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)
    #
    pts_left = np.array([np.transpose(np.vstack([ltx, ploty]))])
    pts_right = np.array([np.flipud(np.transpofse(np.vstack([rtx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    #plus
    mean_x = np.mean((ltx, rtx), axis=0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])
    cv2.fillPoly(color_warp, np.int_([pts]), (216, 168, 74))
    cv2.fillPoly(color_warp, np.int_([pts_mean]), (216, 168, 74))

    #color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, Minv, (Width, Height))
    result = cv2.addWeighted(image, 1, newwarp, 0.4, 0)#0.3-0.4
    #

    return result

if __name__ == '__main__':
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    while not cv_image.size == (Width*Height*3):
        continue

    print("start")
    time.sleep(5)

    while not rospy.is_shutdown():
        image = calibrate_image(cv_image)
        
        #select point
        warp_src, warp_dist = point(warpx_margin_m, warpx_margin)
        
        warp_img, M, Minv = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))
        #warp_process_image(warp_img, last_left_fit, last_right_fit)
        left_fit, right_fit, leftx_current, rightx_current = warp_process_image(warp_img, last_left_fit, last_right_fit)
        lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit)

        cv2.imshow(window_title, lane_img)
        #drive(angle, 20)

        last_left_fit = left_fit
        last_right_fit = right_fit

        warpx_margin_m = leftx_current
        warpx_margin = rightx_current

        cv2.waitKey(1)    
