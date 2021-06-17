#!/usr/bin/env python
import ctypes, os, platform
import numpy as np

obstract_x = None
obstract_y = None
obstract_len = None

crash_x = None
crash_y = None
crash_len = None

platform_chk = str(platform.system())
if platform_chk == "Windows":
    libname = os.path.abspath(os.path.join(os.path.dirname(__file__), "windows/calculator.dll"))
    LIBC = ctypes.WinDLL(libname)
elif platform_chk == "Linux":
    libname = os.path.abspath(os.path.join(os.path.dirname(__file__), "linux/calculator.so"))
    LIBC = ctypes.CDLL(libname)

LIBC.point_location_match.restype = ctypes.c_int
LIBC.cresh.restype = ctypes.c_int
LIBC.arithmetic_mean.restype = ctypes.c_double
LIBC.distance_between_two_points_calculating.restype = ctypes.c_double 
LIBC.rotation_matrix.restype = ctypes.POINTER(ctypes.c_double *2 )
LIBC.steering_to_wheel_ang.restype = ctypes.POINTER(ctypes.c_double * 2)
LIBC.ultrasonic_rtn.restype = ctypes.c_double

def copysign(value, sign):
    return np.copysign(value, sign)

def int_round(number):
    return int(round(number))

def slope_between_two_points_calculating(x1, y1, x2, y2):
    x, y = [x1, x2], [y1, y2]
    coefficient = np.polyfit(x, y, 1)
    return {"m":coefficient[0], "b":coefficient[1]}

def ceil(Float):
    return np.ceil(Float)

def trunc(Float):
    return np.trunc(Float)

def xywh2points(x, y, w, h):
    half_w = float(w) / 2.0
    half_h = float(h) / 2.0
    return [[x-half_w, y-half_h], [x-half_w, y+half_h], [x+half_w, y-half_h], [x+half_w, y+half_h]]

def c_set_map(obstract_x_, obstract_y_):
    global obstract_x, obstract_y, obstract_len
    obstract_x = (ctypes.c_int * len(obstract_x_))(*obstract_x_)
    obstract_y = (ctypes.c_int * len(obstract_y_))(*obstract_y_)
    obstract_len = ctypes.c_int(len(obstract_x_))

def c_set_crash_location(crash_x_, crash_y_):
    global crash_x, crash_y, crash_len
    crash_x = (ctypes.c_int * len(crash_x_))(*crash_x_)
    crash_y = (ctypes.c_int * len(crash_y_))(*crash_y_)
    crash_len = ctypes.c_int(len(crash_x_))

def arithmetic_mean(v1, v2):
    global LIBC
    v1 = ctypes.c_double(v1)
    v2 = ctypes.c_double(v2)
    return float(LIBC.arithmetic_mean(v1, v2))

def distance_between_two_points_calculating(x1, y1, x2, y2):
    global LIBC
    x1 = ctypes.c_double(x1)
    y1 = ctypes.c_double(y1)
    x2 = ctypes.c_double(x2)
    y2 = ctypes.c_double(y2)
    return float(LIBC.distance_between_two_points_calculating(x1, y1, x2, y2))

def rotation_matrix(x, y, ax, ay, angle, return_type):
    global LIBC
    x = ctypes.c_double(float(x))
    y = ctypes.c_double(float(y))
    ax = ctypes.c_double(float(ax))
    ay = ctypes.c_double(float(ay))
    angle = ctypes.c_double(float(angle))
    rtn = LIBC.rotation_matrix(x, y, ax, ay, angle)
    ll = [x for x in rtn.contents]
    return return_type(ll[0]), return_type(ll[1])

def steering_to_wheel_ang_calc(vector, steering_angle, tread, wheel_base, yaw):
    global LIBC
    vector = ctypes.c_char(vector)
    #vector = ctypes.c_char(ord(vector)) #python3
    steering_angle = ctypes.c_double(steering_angle)
    tread = ctypes.c_double(tread)
    wheel_base = ctypes.c_double(wheel_base)
    yaw = ctypes.c_double(yaw)
    rtn = LIBC.steering_to_wheel_ang(vector, steering_angle, tread, wheel_base, yaw)
    ll = [x for x in rtn.contents]
    return float(ll[0]), float(ll[1])

def crash_calc(ax, ay, negative_radian_angle):
    global LIBC, crash_x, crash_y, crash_len, obstract_x, obstract_y, obstract_len
    ax = ctypes.c_int(ax)
    ay = ctypes.c_int(ay)
    negative_radian_angle = ctypes.c_double(negative_radian_angle)
    return bool(LIBC.cresh(crash_x, crash_y, crash_len, ax, ay, negative_radian_angle, obstract_x, obstract_y, obstract_len))

def us_data_receive(x, y, yaw, angle, car_x, car_y, width, height):
    global LIBC, obstract_x, obstract_y, obstract_len
    x = ctypes.c_int(x)
    y = ctypes.c_int(y)
    car_x = ctypes.c_int(int_round(car_x))
    car_y = ctypes.c_int(int_round(car_y))
    yaw = ctypes.c_double(yaw)
    angle = ctypes.c_double(angle)
    width = ctypes.c_int(width)
    height = ctypes.c_int(height)
    rtn = LIBC.ultrasonic_rtn(x, y, yaw, angle, obstract_x, obstract_y, obstract_len, car_x, car_y, width, height)
    return rtn

