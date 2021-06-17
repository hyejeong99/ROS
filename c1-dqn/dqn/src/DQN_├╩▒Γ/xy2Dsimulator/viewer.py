#!/usr/bin/env python

import time
from xycarRL import *

if __name__ == '__main__':
    xycar = learning_xycar(False)
    xycar.set_map("rally_map_d") # snake, square
    xycar.pygame_init()

    #lidar_cnt = 5
    #xycar.set_lidar_cnt(lidar_cnt)
    
    hidden_layer = [256, 256]
    xycar.set_hidden_size(hidden_layer)

    state_select = {
        "car sensor" : True,
        "car yaw" : False,
        "car position" : False,
        "car steer" : True
    }

    xycar.state_setup(state_select)
    
    #xycar.screen_init()
    xycar.ML_init("DQN")

    xycar.set_init_location_pose_random(True) 

    view_epi = 2063

    xycar.load_model(view_epi)

    time.sleep(0.5)

    while (not xycar.pygame_exit):
        state = xycar.episode_init()

        while (xycar.get_episode_done()) or (not xycar.pygame_exit):
            xycar.pygame_exit_chk()
            xycar.calibration_time()
            action = xycar.get_action_viewer(state)
            next_state = xycar.step(action)

            if xycar.get_episode_done():
                break

            state = next_state
            xycar.display_flip()
        
