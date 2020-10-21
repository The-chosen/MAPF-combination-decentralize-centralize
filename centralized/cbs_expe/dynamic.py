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


TIMESTEP_TIME = 3 # Actual consumed time by one agent for one grid step
IS_END = False



class PathPlanner(threading.Thread):
    """
    自定义一个类haha,必须要继承threading.Thread，下面必须要重写一个run()方法。
    把要执行的函数写到run()方法里。如果没有run()方法就会报错。其实这个类的作用就是
    通过haha类里面的run()方法来定义每个启动的子线程要执行的函数内容。
    """
    def __init__(self):
        threading.Thread.__init__(self)


    def run(self):
        global nav, agents, solution, dimensions, static_obstacles, start_time

        while True:
            # Get dynamic obstacles by using the camera
            self.real_pos_dynamic_obs = nav.facility.get_dynamic_obs_position()
            print('[INFO] Position(real) of dynamic obs: ', self.real_pos_dynamic_obs)
            self.coord_pos_dynamic_obs = [nav.real2coord(pos[0], pos[1]) for pos in self.real_pos_dynamic_obs]
            print('[INFO] Position(coord) of static obs: ', self.coord_pos_dynamic_obs)
            obstacles = self.coord_pos_dynamic_obs # Coordinary positions of dynamic obstacles

            conflicts, anytime_limitation = self.calc_conflicts(obstacles, agent_paths)
            anytime_limitation_timestep = (time.time() - start_time) // TIMESTEP_TIME + anytime_limitation

            # Change start point
            agents_cp = deepcopy(agents)
            for agent in agents_cp:
                length = len(solution[agent['name']])
                if anytime_limitation_timestep >= length:
                    temp = solution[agent['name']][length - 1]
                else:
                    temp = solution[agent['name']][anytime_limitation_timestep]
                agent['start'] = [temp['x'], temp['y']]
                    

            env = Environment(dimensions, agents_cp, static_obstacles, obstacles_d=conflicts)
            cbs = CBS(env, anytime_limitation * TIMESTEP_TIME)
            print('[INFO] Start common searching ...')
            print("[INFO] Anytime limitation: " + str(anytime_limitation * TIMESTEP_TIME))
            solution_crr = cbs.search()

            # Write solution to output file (old + crr)

            # If Mover decides to end, then return
            if IS_END:
                return

    '''
    @ Inputs: 1. positions of dynamic obstacles
              2. paths of agents
    @ Outputs: 1. positions of dangerous points
               2. anytime limitaion
    '''
    def calc_conflicts(self, dy_obs_pos, agent_paths):



class Mover(threading.Thread):
    """
    自定义一个类haha,必须要继承threading.Thread，下面必须要重写一个run()方法。
    把要执行的函数写到run()方法里。如果没有run()方法就会报错。其实这个类的作用就是
    通过haha类里面的run()方法来定义每个启动的子线程要执行的函数内容。
    """
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        global nav



if __name__ == "__main__":
    global nav, agents, solution, dimensions, static_obstacles, start_time

    start_time = time.time()

    nav = GoToPoints()

    # Parser part
    parser = argparse.ArgumentParser()
    parser.add_argument("input_pth", help="path of input file containing map and obstacles")
    parser.add_argument("output_pth", help="path of output file with the schedule")
    parser.add_argument("dynamic_obs_pth", help="path of file recording dynamic obs")
    args = parser.parse_args()


    # Read from input file
    print('Read from input ...')
    with open(args.input_pth, 'r') as input_:
        try:
            input_file = yaml.load(input_, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = input_file["map"]["dimensions"]
    static_obstacles = input_file["map"]["obstacles"]
    agents = input_file['agents']

    # Read from output file
    print('Read from output ...')
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
    listener.join()
