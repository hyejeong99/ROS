#!/usr/bin/env python
from calc import *
import numpy as np
import pygame, random

class wheel:
    def __init__(self, x, y, w, h, angle):
        self.position = {"x":x, "y":y, "w":w, "h":h}
        self.current_position = {"x":x, "y":y}
        self.angle = angle

    def calculate_position(self, x, y, yaw):
        X, Y = rotation_matrix(self.position["x"], self.position["y"], x, y, np.radians(-yaw), float)
        self.current_position["x"] = X
        self.current_position["y"] = Y

    def get_information(self):
        return self.current_position["x"], self.current_position["y"], self.position["w"], self.position["h"], self.angle

class distance_device:
    def __init__(self, x, y, angle, device_type):
        self.position = {"x":x, "y":y}
        self.current_position = {"x":x, "y":y}
        self.angle = angle
        self.detect_distance_value = [0.0]
        self.type = device_type

        if self.type == "L":
            self.detect_range = 360
        elif self.type == "U":
            self.detect_range = 15
        
    def get_information(self):
        return self.angle, self.current_position["x"], self.current_position["y"]
        
    def get_distance(self):
        #print(self.detect_distance_value)
        return float(self.detect_distance_value[0])

    def calculate_position(self, x, y, yaw):
        X, Y = rotation_matrix(self.position["x"], self.position["y"], x, y, np.radians(-yaw), float)
        self.current_position["x"] = X
        self.current_position["y"] = Y
        return X, Y

    def dot_bilt(self, screen, x, y, color):
        pygame.draw.circle(screen, color, [int(round(x)), int(round(y))], 3)

    def line_bilt(self, screen, x1, y1, x2, y2, color):
        if (((x1 == -1) and (y1 == -1)) or ((x2 == -1) and (y2 == -1))):
            return
        pygame.draw.line(screen, color, [x1,y1], [x2,y2],3)

class car:
    def __init__(self):
        self.overal_length = 124.0
        self.tread = 60.0
        self.wheel_base = None

        self.steering_angle = 0.0
        self.spatium = 0.0
        self.yaw = 180.0

        self.angular_velocity = 0.0
        self.linear_velocity = 0.0
        self.linear_accelation = 0.0

        self.max_steering_angle = 30.0
        self.max_acceleration = 100.0
        self.mv = 100.0

        self.brake_deceleration = 100.0
        self.free_deceleration = 100.0

        self.accel_padal = False
        self.break_padal = False

        self.init_x = 0.0
        self.init_y = 0.0
        
        self.x = 0.0
        self.y = 0.0

        self.wheel_width = 14
        self.wheel_height = 6

        self.set_init_wheel()

    def set_random(self, r):
        self.random_ = r

    def set_restart(self):
        self.steering_angle = 0.0
        self.spatium = 0.0
        
        self.angular_velocity = 0.0
        self.linear_velocity = 0.0
        self.linear_accelation = 0.0

        self.accel_padal = False
        self.break_padal = False

        self.gear = "D"
        if self.random_:
            self.x = self.init_x[random.randint(0,1)]
            self.y = self.init_y[random.randint(0,1)]
            self.yaw = self.init_yaw[random.randint(0,1)]
        else:
            self.x = self.init_x[0]
            self.y = self.init_y[0]
            self.yaw = self.init_yaw[0]

        self.set_init_wheel()

    def set_init_location(self, x, y, random_):
        self.init_x = x
        self.init_y = y
        self.random_ = random_
        if random_:
            self.x = self.init_x[random.randint(0,1)]
            self.y = self.init_y[random.randint(0,1)]
        else:
            self.x = self.init_x[0]
            self.y = self.init_y[0]

    def set_location(self, x, y):
        self.init_x = x
        self.init_y = y

        self.x = self.init_x[0]
        self.y = self.init_y[0]

    def set_init_yaw(self, yaw, random_):
        self.init_yaw = yaw
        self.random_ = random_

        if random_:
            self.yaw = self.init_yaw[random.randint(0,1)]
        else:
            self.yaw = self.init_yaw[0]

    def set_screen_size(self, w, h):
        self.width = w
        self.height = h

    def set_screen(self, screen):
        self.screen = screen

    def set_init_detect_device_lidar(self):
        x, y = 62, 30
        self.detect_device = [
            distance_device(123-x, 30-y, 0, "L")
        ]
    
    def set_init_detect_device_us(self):
        x, y = 62, 30
        self.detect_device = [
            distance_device( 64-x,  4-y, 90, "U"),
            distance_device(116-x,  8-y, 35, "U"),
            distance_device(123-x, 30-y, 0, "U"),
            distance_device(116-x, 52-y, 360-33, "U"),
            distance_device( 62-x, 56-y, 360-90, "U"),
        ]

    def get_device_distance_data(self):
        distances = []
        for device in self.detect_device:
            d = device.get_distance()
            if d > 111:
                distances.append(444)
                continue
            distances.append(d)
        return distances
    
    def set_init_wheel(self):
        wheel_x = 62
        wheel_y = 30

        self.set_left_front_wheel(0, 99-wheel_x, 2-wheel_y, self.wheel_width, self.wheel_height)
        self.set_left_back_wheel(0, 26-wheel_x, 2-wheel_y, self.wheel_width, self.wheel_height)
        self.set_right_front_wheel(0, 99-wheel_x, 57-wheel_y, self.wheel_width, self.wheel_height)
        self.set_right_back_wheel(0, 26-wheel_x, 57-wheel_y, self.wheel_width, self.wheel_height)

        self.wheel_setup()

    def set_car_crash_location(self, x, y):
        self.crash_location_x = x
        self.crash_location_y = y
        self.set_crash_location(x, y)
    
    def steering_to_wheel_ang(self, vector):
        up_theta, down_theta = steering_to_wheel_ang_calc(vector, self.steering_angle, self.tread, self.wheel_base, self.yaw)
        return up_theta, down_theta

    def set_left_front_wheel(self, angle, x, y, w, h):
        self.left_front_wheel  = wheel(x, y, w, h, angle)

    def get_left_front_wheel(self):
        return self.left_front_wheel.get_information()

    def set_left_back_wheel(self, angle, x, y, w, h):
        self.left_back_wheel = wheel(x, y, w, h, angle)

    def get_left_back_wheel(self):
        return self.left_back_wheel.get_information()

    def set_right_front_wheel(self, angle, x, y, w, h):
        self.right_front_wheel  = wheel(x, y, w, h, angle)

    def get_right_front_wheel(self):
        return self.right_front_wheel.get_information()

    def set_right_back_wheel(self, angle, x, y, w, h):
        self.right_back_wheel  = wheel(x, y, w, h, angle)

    def get_right_back_wheel(self):
        return self.right_back_wheel.get_information()

    def wheel_setup(self):
        left_wheel_base = distance_between_two_points_calculating(self.left_front_wheel.position["x"], \
                                                                  self.left_front_wheel.position["y"], \
                                                                  self.left_back_wheel.position["x"], \
                                                                  self.left_back_wheel.position["y"])

        right_wheel_base = distance_between_two_points_calculating(self.right_front_wheel.position["x"], \
                                                                   self.right_front_wheel.position["y"], \
                                                                   self.right_back_wheel.position["x"], \
                                                                   self.right_back_wheel.position["y"])
        
        self.wheel_base = arithmetic_mean(left_wheel_base, right_wheel_base)

    def update_wheel(self):
        if self.steering_angle > 0:
            far_radius_angle, near_radius_angle = self.steering_to_wheel_ang("L")
            self.left_front_wheel.angle = far_radius_angle
            self.right_front_wheel.angle = near_radius_angle
        elif self.steering_angle < 0:
            far_radius_angle, near_radius_angle = self.steering_to_wheel_ang("R")
            self.right_front_wheel.angle = far_radius_angle
            self.left_front_wheel.angle = near_radius_angle
        else:
            self.right_front_wheel.angle = self.yaw
            self.left_front_wheel.angle = self.yaw

        self.left_back_wheel.angle = self.yaw
        self.right_back_wheel.angle = self.yaw

        X = float(self.x)
        Y = float(self.y)

        self.left_front_wheel.calculate_position(X, Y, self.yaw)
        self.right_front_wheel.calculate_position(X, Y, self.yaw)
        self.left_back_wheel.calculate_position(X, Y, self.yaw)
        self.right_back_wheel.calculate_position(X, Y, self.yaw)

    def update_distance_detect_device(self):
        for device in self.detect_device:
            x, y = device.calculate_position(self.x, self.y, self.yaw)
            distance = us_data_receive(int_round(x), int_round(y), self.yaw, device.angle, self.x, self.y, self.width, self.height)
            device.detect_distance_value = [ distance ]

    def stop(self):
        self.linear_velocity = 0.0
        self.linear_accelation = 0.0
        
    def update(self, dt):
        if self.gear == "D":
            if self.accel_padal:
                if not self.break_padal:
                    self.linear_accelation += (100 * dt)
            else:
                if self.break_padal and (self.linear_velocity > 0):
                    self.linear_accelation = -self.brake_deceleration
                elif self.break_padal and (self.linear_velocity == 0.0) and (self.linear_accelation != 0.0):
                    self.linear_accelation = 0.0
        elif self.gear == "R":
            if self.accel_padal:
                if not self.break_padal:
                    self.linear_accelation -= (100 * dt)
            else:
                if self.break_padal and (self.linear_velocity < 0):
                    self.linear_accelation = self.brake_deceleration
                elif self.break_padal and (self.linear_velocity == 0.0) and (self.linear_accelation != 0.0):
                    self.linear_accelation = 0.0
        else:
            if self.linear_velocity > dt * self.free_deceleration:
                self.linear_accelation = -copysign(self.free_deceleration, self.linear_velocity)

        self.linear_accelation = max(-self.max_acceleration, min(self.linear_accelation, self.max_acceleration))
        self.linear_velocity += (self.linear_accelation * dt)
        self.linear_velocity = min(max(self.min_velocity, self.linear_velocity), self.max_velocity)

        self.angular_velocity = 0.0
        
        if self.steering_angle != 0.0:
            self.angular_velocity = (self.linear_velocity / self.wheel_base) * np.sin(np.radians(self.steering_angle))
        
        self.yaw += (np.degrees(self.angular_velocity) * dt)
        self.spatium = self.linear_velocity * dt
        
        self.x += (self.spatium * np.cos(np.radians(-self.yaw)))
        self.y += (self.spatium * np.sin(np.radians(-self.yaw)))

        self.update_wheel()
        self.update_distance_detect_device()

    def set_minmax_velocity(self, mv):
        self.min_velocity = -mv
        self.max_velocity = mv

    def set_gear(self, gear, mv):
        if (gear == "d") or (gear == "D"):
            gear = "D"

        if (gear == "r") or (gear == "R"):
            gear = "R"

        if (gear == "n") or (gear == "N"):
            gear = "N"

        if (self.linear_velocity != 0.0) and (gear != self.gear):
            return

        if (gear == "D"):
            self.gear = gear
            self.min_velocity = 0.0
            self.max_velocity = mv
        if (gear == "R"):
            self.gear = gear
            self.min_velocity = -mv
            self.max_velocity = 0.0
        if (gear == "N"):
            self.gear = gear
            self.min_velocity = -mv
            self.max_velocity = mv

    def set_steering_angle(self, angle):
        self.steering_angle = max(-self.max_steering_angle, min(angle, self.max_steering_angle))

    def set_accel_padal(self, padal):
        self.accel_padal = padal

    def set_break_padal(self, padal):
        self.break_padal = padal

    def set_velocity(self, v):
        self.linear_velocity = v

    def set_max_velocity(self, m):
        self.min_velocity = -m
        self.max_velocity = m

    def set_accelation(self, a):
        self.linear_accelation = a

    def get_yaw(self):
        return self.yaw

    def get_location(self):
        return self.x, self.y

    def get_steering_angle(self):
        return self.steering_angle

    def set_map(self, obstracle_x, obstracle_y):
        c_set_map(obstracle_x, obstracle_y)

    def set_crash_location(self, crash_location_x, crash_location_y):
        c_set_crash_location(crash_location_x, crash_location_y)

    def crash(self):
        return crash_calc(int_round(self.x), int_round(self.y), np.radians(-self.yaw))

    def border(self, image):
        w, h = image.size
        img = image.load()
        half_w, half_h = float(w)/2.0, float(h)/2.0
        
        X, Y = [], []
        addition = [-1, 0, 1]
        for y in range(h-1):
            for x in range(w-1):
                chk = []
                for ya in addition:
                    for xa in addition:
                        if (y+ya < 0) or (y+ya >= (h-1)):
                            chk.append(0)
                            continue
                        if (x+xa < 0) or (x+xa >= (w-1)):
                            chk.append(0)
                            continue
                        chk.append(img[x+xa, y+ya][3])
                if (chk[4] == 0):
                    del chk[4]
                    for i in chk:
                        if i != 0:
                            X.append(x-half_w)
                            Y.append(y-half_h)
                            break
                if (chk[4] != 0):
                    if ((x == 0) or (y == 0) or (x == (w-2)) or (y == (h-2))):
                        X.append(x-half_w)
                        Y.append(y-half_h)
        
        return np.array(X, dtype=np.int), np.array(Y, dtype=np.int)
