#!/usr/bin/env python

import cv2, glob, copy, os
import numpy as np
from tkinter import *
from tkinter import filedialog

from config import *

viewer = None
line = 0
color = (0,0,0)

def dotLoc(event, x, y, flags, param):
    global line_dot
    global line
    global color

    if event == cv2.EVENT_LBUTTONDOWN:
        line_dot[line]["x"].append(x)
        line_dot[line]["y"].append(y)

def char_search(string, character):
    length = len(string)
    for i in range(length-1, 0, -1):
        if string[i] == character:
            return i

def LINEPOLY_2D(dictionary):
    x = np.array(dictionary["x"])
    y = np.array(dictionary["y"])

    if (len(x) == 0) and (len(y) == 0):
        return 0.0, 0.0, 0.0
    
    coefficient = np.polyfit(y, x, 2)

    return coefficient[0], coefficient[1], coefficient[2]

print("Point image press 1, Look image press 2")
keyInput = input()
if keyInput == "1":
    
    if not os.path.exists("./Image"):
        os.mkdir("./Image")

    if not os.path.exists("./Txt"):
        os.mkdir("./Txt")

    root = Tk()
    root.withdraw()

    dirname = filedialog.askdirectory(initialdir="/home/lhcho/Downloads/aa/clips/0313-1/60", title="Select directory")


    file_list = {}
    for extension in search_extension:
        path = dirname + "/*." + extension
        file_list[extension] = glob.glob(path)

    all_file = []
    for LL in file_list.values():
        all_file += LL

    cv2.namedWindow('image')
    cv2.setMouseCallback('image', dotLoc)

    color = [(0,0,255), (0,255,0), (255,0,0), (255,0,255)]
    pos = ["right-red", "left-green", "right-edge-blue", "left-edge-purple"]

    for f in all_file:
        line_dot = [{"x" : [], "y" : []}, {"x" : [], "y" : []}, {"x" : [], "y" : []}, {"x" : [], "y" : []}]
        image = cv2.imread(f, cv2.IMREAD_COLOR)
        rsz_img = cv2.resize(src=image, dsize=(320, 240), interpolation=cv2.INTER_AREA)

        while True:
            viewer = copy.deepcopy(rsz_img)
            for i in range(4):
                for j in range(len(line_dot[i]["x"])):
                    viewer = cv2.circle(viewer, (line_dot[i]["x"][j],line_dot[i]["y"][j]), 3, color[i], -1)
        
            string = str(line) + ":" + pos[line]
            viewer = cv2.putText(viewer, string, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, color[line])
        
            cv2.imshow("image", viewer)
    
            key = cv2.waitKey(1) & 0xFF
            if key == ord('d'):
                line += 1
                line = min(3, line)
                print(line)
            if key == ord('a'):
                line -= 1
                line = max(0, line)
            if key == ord('c'):
                del_ele = len(line_dot[line]["x"])-1
                if del_ele == -1:
                    continue
                del line_dot[line]["x"][del_ele]
                del line_dot[line]["y"][del_ele]
            if key == 32:
                break

        name_idx = char_search(str(f), "/")
        file_name = str(f)[name_idx+1:]
        ext_idx = char_search(str(file_name), ".")
        file_N = str(file_name)[:ext_idx]

        cv2.imwrite("./Image/"+file_name, rsz_img)
    
        ff = open("./Txt/"+file_N+".txt", "w")
        for k in range(4):
            ff.write(str(k) + " ")
            a, b, c = LINEPOLY_2D(line_dot[k])
            ff.write(str(a) + " ")
            ff.write(str(b) + " ")
            ff.write(str(c) + "\n")
        ff.close()
    

    cv2.destroyAllWindows()
elif keyInput == "2":
    root = Tk()
    root.withdraw()

    init_dir = "/home/" + os.environ["USER"] + "/"
    dirname = filedialog.askdirectory(initialdir=init_dir, title="Select directory")

    if not os.path.exists(dirname+"/Image"):
        print("Image directory not found")
        sys.exit()

    if not os.path.exists(dirname+"/Txt"):
        print("Txt directory not found")
        sys.exit()

    if not os.path.exists(dirname+"/Fail"):
        os.mkdir(dirname+"/Fail")

    img_file_list = glob.glob(dirname + "/Image/*.jpg")
    color = [(0,0,255), (0,255,0), (255,0,0), (255,0,255)]

    list_idx = 0

    while True:
        path = img_file_list[list_idx]
        print(path)
        txt_path = path.replace("/Image/", "/Txt/")[:-3] + "txt"
        if not os.path.isfile(txt_path):
            os.system("touch " + txt_path)
        print(txt_path)
        txt_file = open(txt_path, "r")
        lines = [[],[],[],[]]
    
        for line in txt_file.readlines():
            info = line[:-1].split(" ")
            lines[int(info[0])] = np.array([float(info[1]), float(info[2]), float(info[3])])

        txt_file.close()
    
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        print("image OK")
        for line_idx in range(len(lines)):
            L = lines[line_idx]
            yMax = image.shape[0]    
            y = np.linspace(0, yMax - 1, yMax)
            x = L[0]*y**2 + L[1]*y + L[2]
            for X, Y in zip(x, y):
                if (0 > X) or (X >= 320):
                    continue
                if (0 > Y) or (Y >= 240):
                    continue
                image[int(Y), int(X)] = color[line_idx]

        while True:
            cv2.imshow("view", image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                os.system("rm -rf "+path)
                os.system("rm -rf "+txt_path)
                del img_file_list[list_idx]
                print("Delete Complete")
                break
        
            if key == ord('w'):
                os.system("mv -f "+ path + " " + path.split("/Image/")[0] + "/Fail/")
                os.system("rm -rf "+txt_path)
                del img_file_list[list_idx]
                print("Delete Complete")    
                break

            if key == ord('e'):
                f = open(txt_path, "w")
                print("nono")    
            
                f.close()
                break

            if key == ord('a'):
                list_idx -= 1
                list_idx = min(list_idx, len(img_file_list)-1)
                break

            if key == ord('d'):
                list_idx += 1
                list_idx = max(list_idx, 0)
                break
else:
    print("please press 1 or 2")
