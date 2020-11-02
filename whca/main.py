import numpy as np
from utils import DistanceMethod, CostMapGrid
from visualization import Graph
from node import Node
from agent import Agent
from RRAStar import RRAStar
from whca import WHCA
import random
import copy
import cv2
from visualization import Graph
import time
import argparse
import yaml

TIMESTEP = 3
INITIAL_TIME = 300

def generate_obstacles(shape, number, constraints=[]):
    index_range = shape - 1
    random.seed(1)
    obstacles = copy.deepcopy(constraints)
    positions = []
    for i in range(number):
        x: int
        y: int
        available = False
        while not available:
            x = round(index_range * random.random())
            y = round(index_range * random.random())
            available = True
            for obs in obstacles:
                if obs[0] == x and obs[1] == y:
                    available = False
        positions.append([x, y])
        obstacles.append([x, y])
    return positions

def get_obs_starts_goals_from_file(input_file):
    agents = input_file['agents']
    starts = [agent['start'] for agent in agents]
    goals = [agent['goal'] for agent in agents]
    obstacles = [[obs[0], obs[1]] for obs in input_file['map']['obstacles']]
    shape = input_file['map']['dimensions'][0]
    return shape, obstacles, starts, goals


def calc_path(starts, goals, shape, obstacles, start_time, is_initial = False):

    timestep = TIMESTEP
    if is_initial:
        timestep = INITIAL_TIME

    cost_map = CostMapGrid(shape, obstacles)

    is_time = [0]
    whca = WHCA(starts, goals, cost_map, start_time, timestep, is_time, window=5, visualization=False)
    if is_time[0] == 'TIME':
        return 'TIME'

    print("Start run whca ...")
    agent_solver = whca.run(start_time, timestep)

    if agent_solver == 'TIME':
        return 'TIME'

    trajectories = []
    for i in range(len(agent_solver)):
        # print("agent {}: ".format(i))
        path = []
        for pos in agent_solver[i][0].trajectory:
            path.append(pos.position)
            # print("{}".format(pos.position), end="; / ")
        trajectories.append(copy.deepcopy(path))
        # print(" ")
    print(time.time() - start_time)
    # print('trajectories: ' + str(trajectories))
    return trajectories

def dy_pos_blur(x, y, seed, shape):
    blur_length = 5
    blur_area = [[x + i, y + j] for i in range(- blur_length, blur_length + 1) for j in range(- blur_length, blur_length + 1)]
    
    random.seed(seed)
    while True:
        pt = random.choice(blur_area)
        if (pt[0] >= 0 and pt[0] <= shape - 1) and (pt[1] >= 0 and pt[1] <= shape - 1):
            return pt
    

def replanning():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    parser.add_argument("dynamic_obs", help="dynamic obs")
    # parser.add_argument("dynamic_obs_without", help="dynamic obs")
    parser.add_argument("seed", help="seed for choose blur point")
    # parser.add_argument("result_pth", help="result path")
    args = parser.parse_args()

    with open(args.input, 'r') as input_file:
        try:
            input_ = yaml.load(input_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    
    with open(args.dynamic_obs, 'r') as dynamic_file:
        try:
            dy_obs = yaml.load(dynamic_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    shape, obstacles, starts, goals = get_obs_starts_goals_from_file(input_)

    start_time = time.time()
    initial_pth = calc_path(starts, goals, shape, obstacles, start_time, True)
    if initial_pth == 'TIME':
        print("[WARNING] Time exceed!")

    max_t_agent = max([len(pth) for pth in initial_pth])
    max_t_dy = max(len(pth) for pth in dy_obs['schedule'])
    max_t = max(max_t_agent, max_t_dy)

    solution = {}
    # print(initial_pth)
    for i, pth in enumerate(initial_pth):
        solution['agent' + str(i)] = [{'t': int(str(i)), 'x': int(str(pt[0])), 'y': int(str(pt[1]))} for i, pt in enumerate(pth)]

    print('[INFO] Found solution. ')

    if 'without' not in args.dynamic_obs:

        for t in range(max_t):
            start_time = time.time()

            dy_obs_positions = []
            for pth in dy_obs['schedule'].values():
                if t <= len(pth) - 1:
                    dy_obs_positions.append(dy_pos_blur(pth[t]['x'], pth[t]['y'], args.seed, shape))
                # else:
                #     dy_obs_positions.append(dy_pos_blur(pth[len(pth) - 1]['x'], pth[len(pth) - 1]['y'], args.seed))
            
            new_start = []
            print('Time: ' + str(t))
            for pth in solution.values():
                if t + 1 <= len(pth) - 1:
                    # print(pth)
                    new_start.append([pth[t + 1]['x'], pth[t + 1]['y']])
                else:
                    new_start.append([pth[len(pth) - 1]['x'], pth[len(pth) - 1]['y']])
            print('Start common computing ...')
            traj = calc_path(new_start, goals, shape, obstacles + dy_obs_positions, start_time)
            if traj == 'TIME':
                print('[WARNING] Time exceed! No traj found...')
                continue

            if not traj:
                print("[WARNING] NOT FOUND!")
            else:
                for i, pth in enumerate(solution.values()):
                    if t <= len(pth) - 1:
                        solution['agent' + str(i)] = solution['agent' + str(i)]\
                            [:t + 1] + [{'t': int(str(j + t + 1)), 'x': int(str(pt[0])), 'y': int(str(pt[1]))} for j, pt in enumerate(traj[i])]
                print('[INFO] Found solution. ')

    output = {}
    output['schedule'] = solution

    print(output)

    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml) 
        



def test_whca_grid():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="input file containing map and obstacles")
    # parser.add_argument("output", help="output file with the schedule")
    parser.add_argument("dynamic_obs", help="dynamic obs")
    args = parser.parse_args()

    with open(args.input, 'r') as input_file:
        try:
            input_ = yaml.load(input_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    shape, obstacles, starts, goals = get_obs_starts_goals_from_file(input_)


    start_time = time.time()
    # shape = 32
    display = Graph(shape)
    # obstacles = generate_obstacles(shape, 10)

    # starts = generate_obstacles(shape, 40, obstacles)
    # goals = generate_obstacles(shape, 40, obstacles+starts)
    cost_map = CostMapGrid(shape, obstacles)

    whca = WHCA(starts, goals, cost_map, window=5, visualization=False)
    agent_solver = whca.run()
    end_time = time.time()
    print("[INFO] USE time: " + str(end_time - start_time))

    trajectories = []
    for i in range(len(agent_solver)):
        print("agent {}: ".format(i))
        path = []
        for pos in agent_solver[i][0].trajectory:
            path.append(pos.position)
            print("{}".format(pos.position), end="; / ")
        trajectories.append(copy.deepcopy(path))
        print(" ")

    print('trajectories: ' + str(trajectories))

    # display.show_map(obstacles,starts,goals,trajectories)



if __name__ == "__main__":
    print("start testing")
    # test_RRAStar()
    # test_whca_grid()
    replanning()
