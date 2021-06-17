#!/usr/bin/env python
# -*- coding: utf-8 -*-

import my_reward

from xycarRL import *
from visual import *
    
if __name__ == '__main__':
    xycar = learning_xycar()
    xycar.set_map("rally_map") # snake, square

    hyper_param = {
        "sensor_num" : 5,
        "learning_rate" : 0.001,
        "discount_factor" : 0.98,
        "optimizer_steps" : 300,
        "batch_size" : 64,
        "min_history" : 64,
        "buffer_limit" : 300000,
        "max_episode" : 99999999,
        "update_cycle" : 20,
        "hidden_size" : [256, 256]
    }

    xycar.set_hyperparam(hyper_param)

    state_select = {
        "car sensor" : True,
        "car yaw" : False,
        "car position" : False,
        "car steer" : True
    }

    xycar.state_setup(state_select)
    xycar.Experience_replay_init()
    xycar.ML_init("DQN")

    xycar.set_init_location_pose_random(True) 

    visual = visualize(port=8888)
    visual.chart_init()

    ALL_STEPS, ALL_REWARD = 0, 0
    episode = 0

    while (0 <= episode <= int(hyper_param["max_episode"])):
        episode += 1
        epsilon = max(0.01, 0.3 - 0.01*(float(episode)/150.0))
        xycar.set_E_greedy_func(epsilon)

        state = xycar.episode_init()

        reward, score = 0, 0.0

        while not xycar.get_episode_done():
            action = xycar.get_action(state)
            
            next_state = xycar.step(action)

            if xycar.get_episode_done():                
                for_reward = [
                        xycar.get_episode_total_time(),
                        xycar.get_sensor_value(),
                        xycar.get_xycar_position(),
                        xycar.get_xycar_yaw(),
                        xycar.get_xycar_steering(),
                        xycar.get_step_number()]
                
                reward += my_reward.reward_end_game(for_reward)
                
                xycar.Experience_replay_memory_input(state, action, next_state, reward)

                break

            for_reward = [
                    xycar.get_episode_total_time(),
                    xycar.get_sensor_value(),
                    xycar.get_xycar_position(),
                    xycar.get_xycar_yaw(),
                    xycar.get_xycar_steering(),
                    xycar.get_step_number()]

            reward = my_reward.reward_in_game(for_reward)
            
            xycar.Experience_replay_memory_input(state, action, next_state, reward)
            
            state = next_state
            score += reward

        ALL_STEPS += xycar.get_step_number()
        ALL_REWARD += score

        if xycar.get_max_score() < score:
            xycar.max_score_update(score)
            xycar.model_save(episode)
            xycar.making_video(episode)

        if xycar.get_memory_size() > hyper_param["min_history"]:

            loss = xycar.train(hyper_param["optimizer_steps"])
            visual.loss_graph_update(episode, loss)

        visual.dead_position_update(xycar.get_xycar_position())
        visual.reward_update(episode, score)
        visual.learning_curve_update(episode, ALL_REWARD)

        if (xycar.get_memory_size() > hyper_param["min_history"]) and ((episode % hyper_param["update_cycle"]) == 0) and (episode != 0):
            xycar.mainQ2targetQ()

        if (episode % 10 == 0) and (episode != 0):
            print("episode :{}, memory size : {}, epsilon : {:.1f}%".format(episode, xycar.get_memory_size(), epsilon*100))
