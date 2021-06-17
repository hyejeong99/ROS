#!/usr/bin/env python
#-*- coding: utf-8 -*-

from rendering import render
from map import map
from car import car
from model import *
from collections import OrderedDict, deque
import numpy as np
import time, torch, os, cv2, math

class ReplayBuffer:
    def __init__(self, buffer_limit):
        self.buffer = deque(maxlen=buffer_limit)
    
    def put(self, transition):
        self.buffer.append(transition)
    
    def sample(self, n):
        mini_batch = random.sample(self.buffer, n)
        s_lst, a_lst, r_lst, s_prime_lst, done_mask_lst = [], [], [], [], []
        
        for transition in mini_batch:
            s, a, r, s_prime, done_mask = transition
            s_lst.append(s)
            a_lst.append([a])
            r_lst.append([r])
            s_prime_lst.append(s_prime)
            done_mask_lst.append([done_mask])

        return torch.tensor(s_lst, dtype=torch.float), torch.tensor(a_lst), \
               torch.tensor(r_lst), torch.tensor(s_prime_lst, dtype=torch.float), \
               torch.tensor(done_mask_lst)
    
    def size(self):
        return len(self.buffer)
    
    def free(self):
        self.buffer.clear()

class learning_xycar:
    env = None
    framepersec = 15
    learning_rate = 0
    max_score = 0

    def __init__(self, md=True):
        self.R = render()
        self.M = map()
        self.xycar = car()

        self.end = False
        self.steering_AOC = 200

        self.set_frame(self.framepersec)
        self.curr_path = os.getcwd()
        self.pygame_exit = False
        if md:
            self.make_dir()
            
    def set_map(self, name):
        png = "maps/"+name+".png"
        yml = "maps/"+name+".yaml"
        self.w, self.h, self.conf = self.M.map_read(png, yml)
        self.xymap = self.M.get_map()
        self.xymap_info = self.M.get_map_info()
        self.xymap_obstract = self.M.get_obstracle()

    def set_init_location_pose_random(self, r):
        self.random_init = r
        print(r)
        self.xycar.set_random(r)
        self.xycar.set_init_yaw(self.xymap_info["init_yaw"], self.random_init)
        self.xycar.set_init_location(self.xymap_info["init_position_x"], self.xymap_info["init_position_y"], self.random_init)
        self.xycar.set_init_detect_device_us()

        self.xycar.set_screen_size(self.w, self.h)
        self.M.set_map_obstacle()

    def make_dir(self):
        if not os.path.isdir(self.curr_path+"/video/"):
            os.mkdir(self.curr_path+"/video/")
        if not os.path.isdir(self.curr_path+"/save/"):
            os.mkdir(self.curr_path+"/save/")

    def set_init_gameover(self):
        self.space_bar = False

    def key_event(self):
        key_event = self.R.pygame_set_key_event()
        if key_event[self.R.get_pygame_spacebar_key()]:
            self.space_bar = True

    def get_max_score(self):
        return self.max_score

    def max_score_update(self, score):
        self.max_score = score

    def set_frame(self, fps):
        self.dt = 1.0 / float(fps)

    def set_hyperparam(self, param_dict):
        self.sensor_num = param_dict["sensor_num"]
        self.learning_rate = param_dict["learning_rate"]
        self.discount_factor = param_dict["discount_factor"]
        self.batch_size = param_dict["batch_size"]
        self.min_history = param_dict["min_history"]
        self.buffer_limit = param_dict["buffer_limit"]
        self.hidden_size = param_dict["hidden_size"]

        if self.min_history < self.batch_size:
            self.min_history = self.batch_size

    def state_setup(self, setup_dict):
        self.colorImage, self.car_x_ori, self.car_y_ori = self.R.set_carImg("image/car.png", "image/car.jpg")
        cx, cy = self.xycar.border(self.colorImage)
        self.xycar.set_car_crash_location(cx, cy)
        self.xycar.set_map(self.xymap_obstract[0], self.xymap_obstract[1])
        self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')

        self.car_sensor = setup_dict["car sensor"]
        self.car_yaw = setup_dict["car yaw"]
        self.car_position = setup_dict["car position"]
        self.car_steer = setup_dict["car steer"]

        input_node_cnt = 0

        if self.car_sensor:
            input_node_cnt += 5
        if self.car_yaw:
            input_node_cnt += 1
        if self.car_position:
            input_node_cnt += 2
        if self.car_steer:
            input_node_cnt += 1

        self.input_size = input_node_cnt

    def ML_init(self, model_name):
        print(self.input_size, self.hidden_size, self.learning_rate, model_name)
        study_init(self.input_size, self.hidden_size, self.learning_rate, model_name)
        self.output_size = 3
    
    def Experience_replay_init(self):
        self.memory = ReplayBuffer(self.buffer_limit)

    def Experience_replay_memory_input(self, state, action, next_state, reward):
        done_mask = 0.0 if self.done else 1.0
        self.memory.put((state, action, reward, next_state, done_mask))

    def Experience_replay_close(self):
        self.memory.free()

    def episode_init(self):
        self.step_init()

        self.time_chk = time.time()

        self.steps_beyond_done = None
        self.success = None
        self.exit = False
        self.end = False
        self.xycar.set_gear("n", 100)
 
        self.action = 0
        self.angle = 0
        self.start_time = round(time.time(),2)
        self.chk_time = time.time()

        self.reward = 0
        self.xycar.set_restart()

        self.done = False
        self.step_count = 1
        self.start_time = time.time()

        cnt = 0
        if self.car_sensor:
            cnt += 5
        if self.car_yaw:
            cnt += 1
        if self.car_position:
            cnt += 2
        if self.car_steer:
            cnt += 1

        state = []
        for _ in range(cnt):
            state.append(0.0)

        return np.array(state)

    def pygame_exit_chk(self):
        self.R.pygame_exit_check()
        self.pygame_exit = self.R.exit

    def set_E_greedy_func(self, Egreedy):
        self.E_greedyFunc = Egreedy

    def get_action(self, state):
        return study_get_action(state, self.E_greedyFunc)

    def get_action_viewer(self, state):
        return study_get_action(state)

    def step_init(self):
        self.save_epi = []

    def step_save(self, angle, x, y):
        self.save_epi.append([angle, x, y])

    def snm(self):
        sa = self.xycar.steering_angle
        self.xycar.set_steering_angle(sa)

        x, y = self.xycar.get_location()

        if self.xycar.crash():
            self.end = True
            self.xycar.stop()

        if self.R.rendering:
            self.R.draw_background()
            self.R.draw_map(self.xymap)

            self.R.draw_car(int(x), int(y), self.xycar.get_yaw())

            mx, my, mw, mh, mangle = self.xycar.get_left_front_wheel()
            self.R.draw_wheel(mx, my, mw, mh, mangle)
            mx, my, mw, mh, mangle = self.xycar.get_left_back_wheel()
            self.R.draw_wheel(mx, my, mw, mh, mangle)
            mx, my, mw, mh, mangle = self.xycar.get_right_front_wheel()
            self.R.draw_wheel(mx, my, mw, mh, mangle)
            mx, my, mw, mh, mangle = self.xycar.get_right_back_wheel()
            self.R.draw_wheel(mx, my, mw, mh, mangle)

        self.xycar.update(self.dt)

        self.time = time.time() - self.start_time
        self.viewtime = math.trunc(self.time)

        self.step_save(self.xycar.yaw, self.xycar.x, self.xycar.y)
        info = {"car_sensor": self.xycar.get_device_distance_data(), "car_position":[self.xycar.x, self.xycar.y], "car_yaw":self.xycar.yaw, "car_steer":self.xycar.steering_angle}
        end = self.end
 
        return info, end

    def step_not_move(self):
        info, self.suc_code = self.snm()
        
        self.done = False
        if int(self.suc_code) > 0:
            self.done = True
        
        return np.array(info["car_sensor"])

    def sss(self, action):
        sa = self.xycar.steering_angle
        self.xycar.set_velocity(1000)

        if action == 2:
            sa -= self.steering_AOC * self.dt # Right
        elif action == 0:
            sa += self.steering_AOC * self.dt # Left
        elif action == 1:
            sa = 0

        self.xycar.set_steering_angle(sa)

        info, end = self.snm()
 
        return info, int(end)

    def step(self, action):
        info, self.suc_code = self.sss(action)

        self.done = False
        if int(self.suc_code) > 0:
            self.done = True

        self.distance_device = info["car_sensor"]
        self.xycar_position = info["car_position"]
        self.xycar_yaw = info["car_yaw"]
        self.xycar_steering = info["car_steer"]

        state = []
        if self.car_sensor:
            state += self.distance_device
        if self.car_yaw:
            state += [self.xycar_yaw]
        if self.car_position:
            state += self.xycar_position
        if self.car_steer:
            state += [self.xycar_steering]

        self.step_count += 1

        return np.array(state)

    def train(self, count):
        return study_train(count, self.batch_size, self.discount_factor, self.memory)

    def load_model(self, episode):
        study_model_load(episode)

    def lect_load_model(self, episode):
        study_model_load(episode)

    def set_hidden_size(self, hidden_size):
        self.hidden_size = hidden_size

    def pygame_init(self):
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%i,%i' % (100, 100)
        os.environ['SDL_VIDEO_CENTERED'] = '0'

        self.R.pygame_init()
        self.R.pygame_display_set_caption("simulator")

        self.R.pygame_screen(self.w, self.h)
        self.screen = self.R.screen

        self.xycar.set_screen(self.R.screen)
        
        self.R.pygame_clock()
        self.clock = self.R.clock

    def screen_init(self):
        pass

    def display_flip(self):
        self.R.pygame_display_flip(self.framepersec)

    def calibration_time(self):
        self.dt = self.R.calculate_dt()

    def model_save(self, episode):
        study_model_save(episode)     

    def img_loc(self, angle, x, y):
        X, Y, xycar_img = self.R.rtn_car(x, y, angle)

        rotated = cv2.cvtColor(np.asarray(xycar_img), cv2.COLOR_RGB2BGR)
        pic_pos = [X, Y]

        F = cv2.cvtColor(np.asarray(self.M.map_image), cv2.COLOR_RGB2BGR)

        height = xycar_img.height
        width = xycar_img.width
        for i in range(width):
            for j in range(height):
                if (pic_pos[1]+j >= self.h) or (pic_pos[0]+i >= self.w):
                    continue
                F[pic_pos[1]+j, pic_pos[0]+i] = rotated[j, i]

        return F

    def render(self, epi_num):
        width = self.w
        height = self.h
        FileName = "./video/"+str(epi_num)+".avi"
        out = cv2.VideoWriter(FileName, self.fourcc, float(self.framepersec), (width, height))
 
        all_frame = len(self.save_epi)

        print(" #### start making video : " + str(epi_num) + ".avi #### | frame cnt : " + str(all_frame))
        for i in range(all_frame):
            angle = self.save_epi[i][0]
            x = self.save_epi[i][1]
            y = self.save_epi[i][2]
            
            D = self.img_loc(angle, x, y)

            cv2.putText(D, "episode : "+str(epi_num), (30, 100), cv2.FONT_HERSHEY_DUPLEX, 1, 2, 2)
            out.write(D)

        print(" ##### end making video : " + str(epi_num) + ".avi ##### ")
        out.release()

    def making_video(self, episode):
        self.render(episode)

    def mainQ2targetQ(self):
        study_update()

    def get_episode_total_time(self):
        return time.time() - self.start_time

    def get_sensor_value(self):
        return self.distance_device

    def set_xycar_position(self):
        x_location = [1100.0]
        y_location = [650.0]
        self.xycar.set_location(x_location, y_location)


    def get_xycar_position(self):
        return self.xycar_position

    def get_xycar_yaw(self):
        return self.xycar_yaw

    def get_xycar_steering(self):
        return self.xycar_steering

    def get_step_number(self):
        return self.step_count

    def get_episode_done(self):
        return self.done

    def get_memory_size(self):
        return self.memory.size()

    def get_space_bar_input(self):
        return self.space_bar

