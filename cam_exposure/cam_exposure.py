#!/usr/bin/env python
#-*- coding: utf-8 -*-

############################################################################
#프로그램명 : cam_exposure.py
#작 성 자 : (주)자이트론
#생 성 일 : 2020년 07월 23일                                                
#본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다. 
############################################################################
import os
import cv2
import sys
import rospy
import signal
import numpy as np

from Tkinter import Tk
import tkFileDialog as filedialog
import xml.etree.ElementTree as elemTree
from PIL import Image

device_dir = "/dev/videoCAM"

exposure = 150
exit = False

a = 60
g = 10
margin = 50
b = 2*margin + a

phandok = {"True":"Success", "False":"Fail", "None":""}

suc = None

font = cv2.FONT_HERSHEY_COMPLEX
fontScale = 1.5
thickness = 3

root = Tk()
root.withdraw()

def mouse_click_event_callback(event, x, y, flags, param):
    global up_sig, down_sig, exposure, ext_loc, exit, suc

    if event == cv2.EVENT_LBUTTONDOWN:
        if (y < up_sig[2][1]) and (y > (up_sig[0][0]*x + up_sig[0][1])) and (y > (up_sig[1][0]*x + up_sig[1][1])):
            exposure += 1

        elif (y > down_sig[2][1]) and (y < (down_sig[0][0]*x + down_sig[0][1])) and (y < (down_sig[1][0]*x + down_sig[1][1])):
            exposure -= 1

        elif (ext_loc[0][0] <= x <= ext_loc[1][0]) and (ext_loc[0][1] <= y <= ext_loc[1][1]):
            exit = True

        elif y > (50**2-((x-570)**2))**0.5+350:#+420:
            file_ = filedialog.askopenfilename(initialdir="/home/nvidia/xycar_ws/src", filetypes =[('Launch Files', '*.launch')])
            suc = xml_parser(file_)

def xml_parser(file_path):
    tree = elemTree.parse(file_path)
    root = tree.getroot()
    nodes = root.findall("node")

    usb_cam_node = search_tags(nodes, "usb_cam")

    if usb_cam_node == None:
        return False

    params = usb_cam_node.findall("param")
    rtn = search_tags(params, "exposure")
    rtn.set("value", str(exposure))

    tree.write(file_path)
    return True

def exposure_set(value):
    global device_dir

    if str(type(value)) != "<type 'int'>":
        return

    command = "v4l2-ctl -d "+ device_dir +" -c exposure_absolute=" 
    command += str(value)
    os.system(command)

def exposure_range(value):
    value = max(min(value, 5000), 1)
    return value

def search_tags(nodes, name):
    return_node = None
    for node in nodes:
        for item in node.items():
            if (item[0] == 'name') and (item[1] == name):
                return_node = node
                break
        if return_node != None:
            break
    return return_node

def inner_position(param):
    x1 = param[0]
    y1 = param[1]
    x2 = param[2]
    y2 = param[3]

    m = float(y2 - y1) / float(x2 - x1)
    b = y1 - (m*x1)
    return (m, b)

def list2param(array1, array2):
    return (array1[0], array1[1], array2[0], array2[1])

def triangle(array):
    one = inner_position(list2param(array[0], array[1]))
    two = inner_position(list2param(array[0], array[2]))
    three = inner_position(list2param(array[1], array[2]))
    return one, two, three

cam = cv2.VideoCapture(device_dir, cv2.CAP_V4L2)

exposure_set(exposure)

cv2.namedWindow('xytron')
cv2.setMouseCallback('xytron', mouse_click_event_callback)

#up = np.array([[60, a], [10, a + margin], [110, a + margin]], np.int32)
up = np.array([[60, a-20], [5, a + margin], [120, a + margin]], np.int32)
up_sig = triangle(up)

#down = np.array([[60, b+g], [10, b-margin+g], [110, b-margin+g]], np.int32)
down = np.array([[60, b+g], [5, b-margin+g-20], [120, b-margin+g]], np.int32)
down_sig = triangle(down)

#ext_loc = [[600,0], [640, 40]]
ext_loc = [[550,10], [620, 80]]

#exit
vposE=10
hposE=550
exitI = cv2.imread("cancel2.png")
exitI = cv2.resize(exitI, (70, 70))
rE, cE, chE = exitI.shape
img2grayE = cv2.cvtColor(exitI, cv2.COLOR_BGR2GRAY)
retE, maskE = cv2.threshold(img2grayE, 10, 255, cv2.THRESH_BINARY_INV)
mask_invE = cv2.bitwise_not(maskE)
exit_bw = cv2.bitwise_and(exitI, exitI, mask=maskE)
#save
vposS=400
hposS=550
saveI = cv2.imread("save2.png")
saveI = cv2.resize(saveI, (70, 70))
rS, cS, chS = saveI.shape
img2gray = cv2.cvtColor(saveI, cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY_INV)
mask_inv = cv2.bitwise_not(mask)
save_bw = cv2.bitwise_and(saveI, saveI, mask=mask)
#up
vposU=40
hposU=30
upI = cv2.imread("up.png")
upI = cv2.resize(upI, (60, 60))
rU, cU, chU = upI.shape
img2grayU = cv2.cvtColor(upI, cv2.COLOR_BGR2GRAY)
retU, maskU = cv2.threshold(img2grayU, 10, 255, cv2.THRESH_BINARY_INV)
mask_invU = cv2.bitwise_not(maskU)
up_bw = cv2.bitwise_and(upI, upI, mask=maskU)
#down
vposD=100
hposD=30
downI = cv2.imread("down.png")
downI = cv2.resize(downI, (60, 60))
rD, cD, chD = downI.shape
img2grayD = cv2.cvtColor(downI, cv2.COLOR_BGR2GRAY)
retD, maskD = cv2.threshold(img2grayD, 10, 255, cv2.THRESH_BINARY_INV)
mask_invD = cv2.bitwise_not(maskD)
down_bw = cv2.bitwise_and(downI, downI, mask=maskD)

while cam.isOpened():
    ret, frame = cam.read()
    if exit:
        break
    if not ret:
        continue

    exposure = exposure_range(exposure)
    exposure_set(exposure)
    
    #up
    roi = frame[vposU:rU+vposU, hposU:cU+hposU]
    frame_bwU = cv2.bitwise_and(roi, roi, mask=mask_invU)
    dst = cv2.add(frame_bwU, up_bw)
    frame[vposU:rU+vposU, hposU:cU+hposU] = dst
    #down
    roi = frame[vposD:rD+vposD, hposD:cD+hposD]
    frame_bwD = cv2.bitwise_and(roi, roi, mask=mask_invD)
    dst = cv2.add(frame_bwD, down_bw)
    frame[vposD:rD+vposD, hposD:cD+hposD] = dst
    #exit
    roi = frame[vposE:rE+vposE, hposE:cE+hposE]
    frame_bwE = cv2.bitwise_and(roi, roi, mask=mask_invE)
    dst = cv2.add(frame_bwE, exit_bw)
    frame[vposE:rE+vposE, hposE:cE+hposE] = dst
    #save
    roi = frame[vposS:rS+vposS, hposS:cS+hposS]
    frame_bwS = cv2.bitwise_and(roi, roi, mask=mask_inv)
    dst = cv2.add(frame_bwS, save_bw)#updown
    frame[vposS:rS+vposS, hposS:cS+hposS] = dst
    #end
    
    #exposure text
    frame = cv2.putText(frame, str(exposure), (10,40), font, fontScale, (0,0,255), thickness)

    pos_x = 110
    frame = cv2.putText(frame, phandok[str(suc)], (pos_x, 300), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255),7)
    if suc != None:
        if not suc:
            pos_x *= 2
    cv2.imshow('xytron', frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()
