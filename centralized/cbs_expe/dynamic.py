#!/usr/bin/env python3

from utilities.transformations import *
from utilities.graph import *
from pios_facility import PiosFacility
import numbers as np
from get_init import pios_facility_parameter
import matplotlib.pyplot as plt
from utilities.barrier_certificates import *
from utilities.misc import *
from utilities.controllers import *
import time
from pynput import keyboard
import itertools
import random
from navigationInterface import NavigationAdapter, Observation, ObservationAdapter, MotionSolutionAdapter
import sys
import yaml
import numpy as np
from algorithm.multi_thread_mapf import *
from algorithm.cbs_mine import Environment, CBS
from algorithm.a_star_mine import *
from go_to_points import GoToPoints
import threading
import argparse
from copy import deepcopy

TIMESTEP_TIME = 5 # Actual consumed time by one agent for one grid step
IS_END = False
THRESHOLD = 25 # Threshold for dangerous points acknowledgement
R = 4
INF_NUM = 999999999999999

class Utils(object):
    def __init__(self):
        pass

    '''
    @Params:
    pos_pt: Position of the point.
    pos_obs: Position of the obstacle.
    pos_agent: Position of the agent.
    @Return:
    Score of the point.
    '''
    def score_func(self, pos_pt, pos_obs, pos_agent):
        dist = abs(pos_pt['x'] - pos_obs[0]) + abs(pos_pt['y'] - pos_obs[1])
        cost = abs(pos_pt['x'] - pos_agent['x']) + abs(pos_pt['y'] - pos_agent['y'])
        score = 0
        if dist <= 1:
            score = INF_NUM
        elif dist <= 3:
            score = THRESHOLD * 1
        elif dist == 4:
            score = THRESHOLD * 0.75
        elif dist == 5:
            score = THRESHOLD * 0.5
        elif dist >= 6:
            score = THRESHOLD * 0.1

        if cost <= 3:
            score *= 1
        elif cost == 4:
            score *= 0.8
        elif cost == 5:
            score *= 0.5
        elif cost == 6:
            score *= 0.2
        elif cost >= 7:
            score *= 0.1
        return int(score)
        

    '''
    @Params:
    t_pos: timestep of the point.
    t_crr: timestep of current agent.
    '''
    def anytime_func(self, cost):
        # return cost // 2
        return cost - 1



class PathPlanner(threading.Thread):
    """
    自定义一个类haha,必须要继承threading.Thread，下面必须要重写一个run()方法。
    把要执行的函数写到run()方法里。如果没有run()方法就会报错。其实这个类的作用就是
    通过haha类里面的run()方法来定义每个启动的子线程要执行的函数内容。
    """
    def __init__(self):
        threading.Thread.__init__(self)
        self.utils = Utils()


    def run(self):
        global nav, agents, solution, dimensions, static_obstacles, start_time, output_pth, crr_timestep

        while True:
            # Get dynamic obstacles by using the camera
            self.real_pos_dynamic_obs = nav.facility.get_dynamic_obs_position()
            print('[INFO] Position(real) of dynamic obs: ', self.real_pos_dynamic_obs)
            self.coord_pos_dynamic_obs = [nav.real2coord(pos[0], pos[1]) for pos in self.real_pos_dynamic_obs]
            print('[INFO] Position(coord) of static obs: ', self.coord_pos_dynamic_obs)
            obstacles = self.coord_pos_dynamic_obs # Coordinary positions of dynamic obstacles

            conflicts, anytime_limitation = self.calc_conflicts(obstacles)
            anytime_limitation_timestep = crr_timestep + anytime_limitation

            # Change start point
            agents_cp = deepcopy(agents)
            for agent in agents_cp:
                length = len(solution[agent['name']])
                if anytime_limitation_timestep >= length:
                    temp = solution[agent['name']][length - 1]
                else:
                    temp = solution[agent['name']][anytime_limitation_timestep]
                agent['start'] = [temp['x'], temp['y']]
                    

            # Start updating paths using BCBS
            env = Environment(dimensions, agents_cp, static_obstacles, obstacles_d=conflicts)
            cbs = CBS(env, anytime_limitation * TIMESTEP_TIME)

            compute_start_time = time.time()
            print('[INFO] Start common searching ...')
            print("[INFO] Anytime limitation: " + str(anytime_limitation * TIMESTEP_TIME))
            solution_crr = cbs.search()

            # combine previous solution [:timestep + anytime_limitation] and new solution
            if not solution_crr:
                print('[ERROR] Solution not found!')
                continue
            print('[INFO] Common searching ends')

            # Get previous solution
            solution_pre = solution

            # util: map function
            def f(x):
                x['t'] += anytime_limitation_timestep
                return x

            for agent in solution_pre.keys():
                solution_crr[agent] = solution_pre[agent][:anytime_limitation_timestep] + (list(map(f, solution_crr[agent]))) 

            solution = solution_crr
            print('[INFO] COMMON SOLUTION:')
            print(solution)

            compute_end_time = time.time()
            print('[INFO] Common searching use time: ' + str(compute_end_time - compute_start_time))

            # Write solution to output file (old + crr)
            output = {}
            output["schedule"] = solution
            # output["cost"] = env.compute_solution_cost(solution)
            with open(output_pth, 'w') as output_yaml:
                yaml.safe_dump(output, output_yaml) 


            # If Mover decides to end, then return
            if IS_END:
                return

    '''
    @ Inputs: 1. positions of dynamic obstacles
              2. paths of agents
    @ Outputs: 1. positions of dangerous points
               2. anytime limitaion
    '''
    def calc_conflicts(self, dy_obs_pos):
        global solution
        conflicts = []
        anytime_limitaion = []
        # Iterate each agent. And iterate points of each agent's path
        for agent_name in solution.keys():
            agent_path = solution[agent_name]
            agent_pos = agent_path[0]
            
            for point in agent_path:
                point_score = 0
                if agent_pos == point:
                    continue
                for dy_obs in dy_obs_pos:
                    if not (dy_obs[0] < R + agent_pos['x'] and dy_obs[0] > - R + agent_pos['x'] \
                        and dy_obs[1] < R + agent_pos['y'] and dy_obs[1] > - R + agent_pos['y']):
                        continue
                    point_score += self.utils.score_func(point, dy_obs, agent_pos)
                point['score'] = point_score
                if point_score >= THRESHOLD:
                    conflicts.append((point['x'], point['y']))
                    anytime_limitaion.append(point['t'] - agent_pos['t'])
        return conflicts, min(anytime_limitaion)

class Mover(threading.Thread):
    """
    自定义一个类haha,必须要继承threading.Thread，下面必须要重写一个run()方法。
    把要执行的函数写到run()方法里。如果没有run()方法就会报错。其实这个类的作用就是
    通过haha类里面的run()方法来定义每个启动的子线程要执行的函数内容。
    """
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        global nav, agents, solution, dimensions, static_obstacles, start_time, output_pth, crr_timestep
        num_agents = len(solution.keys())
        crr_timestep = 0
        

        while True:
            start_time_timestep = time.time()

            # Judege whether end
            total_time = max([len(solution[agent_name]) for agent_name in solution.keys()])
            if crr_timestep + 1 == total_time:
                IS_END = True
                return
            
            # Add the next timestep
            x = np.array([])
            y = np.array([])
            z = np.zeros(num_agents)
            for agent in range(num_agents):
                agent_name = 'agent' + str(agent)
                agent_traj = solution[agent_name]
                if crr_timestep >= len(agent_traj):
                    x_agent = agent_traj[len(agent_traj) - 1]['x']
                    y_agent = agent_traj[len(agent_traj) - 1]['y']
                else:
                    x_agent = agent_traj[crr_timestep]['x']
                    y_agent = agent_traj[crr_timestep]['y']

                x_agent, y_agent = nav.coord2real(x_agent, y_agent)
                x = np.append(x, x_agent)
                y = np.append(y, y_agent)
            point = np.array([x, y, z])   

            # Actual moving agents
            nav.move_to(point)

            # Ensure every timestep the same length
            end_time_timestep = time.time()
            consume_time = end_time_timestep - start_time_timestep
            if consume_time > TIMESTEP_TIME:
                print("[ERROR] " + crr_timestep + " timestep consume time exceed!!! Use time: " + str(consume_time))
            else:
                print("[INFO] Sleeping " + str(consume_time - TIMESTEP_TIME))
                time.sleep(consume_time - TIMESTEP_TIME)

            crr_timestep += 1




if __name__ == "__main__":
    global nav, agents, solution, dimensions, static_obstacles, start_time, output_pth

    start_time = time.time()

    nav = GoToPoints()

    # Parser part
    parser = argparse.ArgumentParser()
    parser.add_argument("input_pth", help="path of input file containing map and obstacles")
    parser.add_argument("output_pth", help="path of output file with the schedule")
    parser.add_argument("dynamic_obs_pth", help="path of file recording dynamic obs")
    args = parser.parse_args()

    # Detect static obstacles and calculate initial paths
    nav.calc_initial_traj(args.input_pth, args.output_pth)

    # Read from input file
    print('[INFO] Read from input ...')
    with open(args.input_pth, 'r') as input_:
        try:
            input_file = yaml.load(input_, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = input_file["map"]["dimensions"]
    static_obstacles = input_file["map"]["obstacles"]
    agents = input_file['agents']

    # Read from output file
    print('[INFO] Read from output ...')
    output_pth = args.output_pth
    with open(args.output_pth, 'r') as output_:
        try:
            output_file = yaml.load(output_, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    solution = output_file['schedule']

    listener = keyboard.Listener(on_press=nav.on_press)
    listener.start()
    if len(sys.argv) > 1:
        if sys.argv[1] == 'r':
            print('rectangular_formation')
            nav.rectangular_formation()
        if sys.argv[1] == 'v':
            print('v_formation')
            nav.v_formation()
        if sys.argv[1] == 'b':
            print('backend')
            nav.backend()
        if sys.argv[1] == 't':
            print('head to head')
            nav.head_to_head()
        if sys.argv[1] == 'h':
            print('home')
            nav.go_home()
        if sys.argv[1] == 'i':
            print('forward')
            nav.follow_straight_line()
        if sys.argv[1] == 'y':
            # nav.go_mine_try()
            nav.move_traj()
        if sys.argv[1] == 'm':
            nav.get()
        if sys.argv[1] == 'q':
            threads = [PathPlanner(), Mover()]
            # Start threads
            for thr in threads:
                thr.start()

            for thr in threads:
                if thr.is_alive():
                    thr.join()
            
    listener.join()
