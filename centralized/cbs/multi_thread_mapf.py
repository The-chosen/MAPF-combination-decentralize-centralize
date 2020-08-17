# -- coding:UTF-8 --
"""

Python implementation of decentralized update algorithm with multi threads

author: Yue Yang (@The-chosen)

"""

import sys
sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy
from cbs.a_star_mine import AStar

import time
import eventlet
from cbs.cbs_mine import *
import threading
from queue import PriorityQueue

# eventlet.monkey_patch()

# Global variables
pq = PriorityQueue()
solution = None
alive_agent_thread_num = None
DYNAMIC_OBSTACLES = None

INF_NUM = 999999999999999
R = 4
THRESHOLD = 25
TIME_LIMIT = 20

class Server(threading.Thread):
    """
    自定义一个类haha,必须要继承threading.Thread，下面必须要重写一个run()方法。
    把要执行的函数写到run()方法里。如果没有run()方法就会报错。其实这个类的作用就是
    通过haha类里面的run()方法来定义每个启动的子线程要执行的函数内容。
    """
    def __init__(self, dimension, obstacles, agents, output_file, num_combine=2):
        threading.Thread.__init__(self)
        self.dimension = dimension
        self.obstacles = obstacles
        self.agents = agents

        self.output_file = output_file
        self.num_combine = num_combine # Number of conflict list to be combined

    def run(self):
        global alive_agent_thread_num, solution, pq, TIME_LIMIT
        
        # Robots start
        while True:
            # agents all finish their paths
            # print("alive_agent_thread_num: ", alive_agent_thread_num)
            if alive_agent_thread_num == 0:
                print(solution)
                output = {}
                output["schedule"] = solution
                # output["cost"] = env.compute_solution_cost(solution)
                with open(self.output_file, 'w') as output_yaml:
                    yaml.safe_dump(output, output_yaml) 
                return

            # print(pq)
            if pq.empty():
                continue
            else:
                # print('happy')
                compute_start_time = time.time()
                # Combine several conflict list & get minimum anytime limitation
                conflicts = []
                min_anytime_limitation_timestep = INF_NUM
                for i in range(self.num_combine):
                    if pq.empty():
                        break
                    _, conflict, anytime_limitation_timestep = pq.get()
                    conflicts += conflict
                    if anytime_limitation_timestep < min_anytime_limitation_timestep:
                        min_anytime_limitation_timestep = anytime_limitation_timestep
                # change agents start time
                agents_cp = deepcopy(self.agents)
                for agent in agents_cp:
                    length = len(solution[agent['name']])
                    if min_anytime_limitation_timestep >= length:
                        temp = solution[agent['name']][length - 1]
                    else:
                        temp = solution[agent['name']][min_anytime_limitation_timestep]
                    agent['start'] = [temp['x'], temp['y']]

                env = Environment(self.dimension, agents_cp, self.obstacles, obstacles_d=conflicts)
                # Searching
                cbs = CBS(env, is_add_constraint=False)
                print('[INFO] Start common searching ...')
                # with eventlet.Timeout(TIME_LIMIT, False):
                solution_crr = cbs.search()
                if not solution_crr:
                    print('[ERROR] Solution not found!')
                    continue
                print('[INFO] Common searching ends')
                # Get previous solution
                solution_pre = solution

                # util: map function
                def f(x):
                    x['t'] += min_anytime_limitation_timestep
                    return x

                # combine previous solution [:timestep + anytime_limitation] and new solution
                for agent in solution_pre.keys():
                    # print("solution_crr: " + str(solution_crr))
                    # print("solution_pre: " + str(solution_pre))
                    solution_crr[agent] = solution_pre[agent][:min_anytime_limitation_timestep] + (list(map(f, solution_crr[agent]))) 

                solution = solution_crr
                compute_end_time = time.time()
                print('Common searching use time: ' + str(compute_end_time - compute_start_time))

class Agent(threading.Thread):
    '''
    @Params:
    agent_name: name of the agent.
    R: Range the agent can see. It's a square(2R + 1 * 2R + 1).
    paths: Paths of all the agents.
    threshold: If score of one point is larger than this parameter, the point will be added to constraint list

    timestep_time: time limitation for each timestep
    '''
    def __init__(self, agent_name, timestep_time=5, R=R, threshold=THRESHOLD):
        global solution
        threading.Thread.__init__(self)
        self.R = R
        self.agent_name = agent_name
        self.paths = deepcopy(solution)
        self.threshold = threshold
        self.timesteps = len(self.paths[self.agent_name])

        self.timestep_time = timestep_time

        self.detect_pt = []
        self.detect_obstacle = []

        self.utils = Utils()

    def run(self):
        global alive_agent_thread_num, solution
        for timestep in range(self.timesteps):
            self.paths = deepcopy(solution)
            self.do_things_in_one_timestep(timestep)
        alive_agent_thread_num -= 1
        print('[INFO] ' + str(self.agent_name) + ' end!')


    # @return: (conflicts[], anytime_limitation + timestep)
    def do_things_in_one_timestep(self, timestep):
        global pq
        start_time = time.time()

        # ↓↓↓↓↓ Do things from here ↓↓↓↓↓
        points = self.detect(timestep, DYNAMIC_OBSTACLES)
        if not points:
            end_time = time.time()
            use_time = end_time - start_time
            time.sleep(self.timestep_time - use_time) # ensure the thing will be done within timestep_time
            real_end = time.time()
            print(self.agent_name + ' finish timestep ' + str(timestep) + '. Use time: ' + str(real_end - start_time) + '. Have collision points.')
            return
        conflict_list_agent, anytime_limitation, min_cost_pt = self.find_constraint_list()
        pq.put((min_cost_pt, conflict_list_agent, anytime_limitation + timestep))
        
        # ↑↑↑↑↑ End things from above ↑↑↑↑↑

        end_time = time.time()
        use_time = end_time - start_time
        time.sleep(self.timestep_time - use_time) # ensure the thing will be done within timestep_time
        real_end = time.time()
        print(self.agent_name + ' finish timestep ' + str(timestep) + '. Use time: ' + str(real_end - start_time) + '. No collision points.')

        
        

    '''
    @Params:
    t: current timestep

    @Return:
    scored list of points

    @Exlanation: In reality, these should be a function to accept data of sensors. 
                Need to judge whether the occupied thing is obstacle or not.
    '''
    def detect(self, t, dynamic_obstacles):
        self.crr_t = t
        # Form the detection square
        # print(self.paths.keys())
        if len(self.paths[self.agent_name]) <= t: # Have reached end point
            return False
        central_pos = self.paths[self.agent_name][t]
        x_right = central_pos['x'] + R
        x_left = central_pos['x'] - R
        y_up = central_pos['y'] + R
        y_bottom = central_pos['y'] - R


        # Find points of path that are in the detection square
        for agent_name, path in self.paths.items():
            if t >= len(path): # No action after t for this agent, then it will be static and detect obstacles
                crr_pos = path[len(path) - 1]
            else:
                crr_pos = path[t]
            for i in range(t + 1, len(path)):
                pos = path[i]
                if (pos['x'] <= x_right) and (pos['x'] >= x_left) and (pos['y'] <= y_up) and (pos['y'] >= y_bottom):
                    pos['distance'] = i - t
                    pos['agent_name'] = agent_name
                    pos['crr_pos'] = crr_pos
                    self.detect_pt.append(pos)


        # Find obstacles that are dangerous and calculate the score of every point
        is_dangerous_obs = False
        for obstacle_name, path in dynamic_obstacles.items():
            is_obstacle_static = False
            if t >= len(path): # No action after t for this obstacle
                pos = path[len(path) - 1]
                is_obstacle_static = True
            else:
                pos = path[t] # pos: position of obstacle
            if (pos['x'] <= x_right) and (pos['x'] >= x_left) and (pos['y'] <= y_up) and (pos['y'] >= y_bottom):
                for pt in self.detect_pt: # pt: point on the path
                    # For stop dynamic obstacles, only when it's on path will be considered.
                    if is_obstacle_static:
                        if pos['x'] == pt['x'] and pos['y'] == pt['y']: 
                            is_dangerous_obs = True
                            if 'score' not in pt.keys():
                                pt['score'] = 0
                            pt['score'] += self.utils.score_func(pt, pos, pt['crr_pos'])
                        continue
                    # Else, use score function to calculate how dangerous it is
                    if abs(pos['x'] - pt['x']) + abs(pos['y'] - pt['y']) <= pt['distance']: # bool to decide whether dangerous
                        # print(pos)
                        # print(pt)
                        is_dangerous_obs = True
                        if 'score' not in pt.keys():
                            pt['score'] = 0
                        pt['score'] += self.utils.score_func(pt, pos, pt['crr_pos'])
        
        if len(self.detect_pt) == 0 or not is_dangerous_obs:
            # print('No point or dangerous obstacles detected!')
            return False
        
        return self.detect_pt
    
    '''
    @Params: 
    None
    @Return:
    Constraint dictionary: 
    {'agent2': <__main__.Constraints object at 0x7fa54c69c2b0>, 
    'agent7': <__main__.Constraints object at 0x7fa54c69c2b0>}
    '''
    def find_constraint_list(self):
        constraint_dict = {}
        conflict_list = []
        min_cost_pt = INF_NUM
        for pt in self.detect_pt:
            if 'score' not in pt.keys():
                continue
            if pt['score'] >= self.threshold:
                if pt['t'] - self.crr_t < min_cost_pt: # for calculating anytime
                    min_cost_pt = pt['t'] - self.crr_t
                conflict_list.append((pt['x'], pt['y']))
                # v_constraint = VertexConstraint(pt['t'], Location(pt['x'], pt['y']))
                # if pt['agent_name'] in constraint_dict.keys():
                #     constraint_dict[pt['agent_name']].vertex_constraints |= {v_constraint}
                # else:
                #     constraint = Constraints()
                #     constraint.vertex_constraints |= {v_constraint}
                #     constraint_dict[pt['agent_name']] = constraint
        anytime_limitation = self.utils.anytime_func(min_cost_pt)
        return conflict_list, anytime_limitation, min_cost_pt


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
        dist = abs(pos_pt['x'] - pos_obs['x']) + abs(pos_pt['y'] - pos_obs['y'])
        cost = abs(pos_pt['x'] - pos_agent['x']) + abs(pos_pt['y'] - pos_agent['y'])
        score = 0
        if dist <= 1:
            score = INF_NUM
        elif dist == 2:
            score = THRESHOLD * 1
        elif dist == 3:
            score = THRESHOLD * 0.5
        elif dist == 4:
            score = THRESHOLD * 0.25
        elif dist >= 5:
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
        return cost // 2

def main():
    global solution, alive_agent_thread_num, DYNAMIC_OBSTACLES

    # Parser part
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()
    
    # Read from input file
    print('Read from input ...')
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    obstacles = param["map"]["obstacles"]
    agents = param['agents']

    # Initial searching 
    env = Environment(dimension, agents, obstacles)
    cbs = CBS(env)
    print('[INFO] Start initial searching ...')
    solution = cbs.search()
    print('[INFO] Initial searching end')

    if not solution:
        print("[ERROR] Initial solution not found" ) 
        return

    # Assign value to global variables(alive_agent_thread_num & DYNAMIC_OBSTACLES)
    alive_agent_thread_num = len(agents)
    with open('dynamic_obstacle_pth.yaml', 'r') as d_obstacles_file:
        try:
            DYNAMIC_OBSTACLES = yaml.load(d_obstacles_file, Loader=yaml.FullLoader)['schedule']
        except yaml.YAMLError as exc:
            print(exc)

    # Create threads including server and agents
    threads = []
    server_thread = Server(dimension, obstacles, agents, args.output) # create server thread
    threads.append(server_thread)
    for agent in agents:
        agent_name = agent['name']
        agent_thread = Agent(agent_name) # create agent thread
        threads.append(agent_thread)

    # Start threads
    for thr in threads:
        thr.start()

    for thr in threads:
        if thr.is_alive():
            thr.join()

if __name__=='__main__':
    main()