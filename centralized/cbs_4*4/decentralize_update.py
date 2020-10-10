"""

Python implementation of decentralized update algorithm

author: Yue Yang (@The-chosen)

"""

import sys
sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy
from cbs.a_star import AStar

import time
from cbs.cbs import *


DYNAMIC_OBSTACLES = None
R = 4
THRESHOLD = 20
INF_NUM = 999999999999999

class Agent(object):
    '''
    @Params:
    agent_name: name of the agent.
    R: Range the agent can see. It's a square(2R + 1 * 2R + 1).
    paths: Paths of all the agents.
    threshold: If score of one point is larger than this parameter, the point will be added to constraint list
    '''
    def __init__(self, agent_name, paths, R=R, threshold=THRESHOLD):
        self.R = R
        self.agent_name = agent_name
        self.paths = paths
        self.threshold = threshold

        self.detect_pt = []
        self.detect_obstacle = []

        self.utils = Utils()

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
                            pt['score'] += self.utils.score_func(pt, pos)
                        continue
                    # Else, use score function to calculate how dangerous it is
                    if abs(pos['x'] - pt['x']) + abs(pos['y'] - pt['y']) <= pt['distance']: # bool to decide whether dangerous
                        # print(pos)
                        # print(pt)
                        is_dangerous_obs = True
                        if 'score' not in pt.keys():
                            pt['score'] = 0
                        pt['score'] += self.utils.score_func(pt, pos)
        
        if len(self.detect_pt) == 0 or not is_dangerous_obs:
            print('No point or dangerous obstacles detected!')
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
        min_cost_pt = INF_NUM
        for pt in self.detect_pt:
            if 'score' not in pt.keys():
                continue
            if pt['score'] >= self.threshold:
                if pt['t'] - self.crr_t < min_cost_pt:
                    min_cost_pt = pt['t'] - self.crr_t
                v_constraint = VertexConstraint(pt['t'], Location(pt['x'], pt['y']))
                if pt['agent_name'] in constraint_dict.keys():
                    constraint_dict[pt['agent_name']].vertex_constraints |= {v_constraint}
                else:
                    constraint = Constraints()
                    constraint.vertex_constraints |= {v_constraint}
                    constraint_dict[pt['agent_name']] = constraint
        anytime_limitation = self.utils.anytime_func(min_cost_pt)
        return constraint_dict, anytime_limitation


class Utils(object):
    def __init__(self):
        pass

    '''
    @Params:
    pos_pt: Position of the point.
    pos_obs: Position of the obstacle.
    @Return:
    Score of the point.
    '''
    def score_func(self, pos_pt, pos_obs):
        dist = abs(pos_pt['x'] - pos_obs['x']) + abs(pos_pt['y'] - pos_obs['y'])
        score = 0
        if dist <= 2:
            score = INF_NUM
        elif dist == 3:
            score = THRESHOLD * 1
        elif dist == 4:
            score = THRESHOLD * 0.5
        elif dist == 5:
            score = THRESHOLD * 0.25
        elif dist >= 6:
            score = THRESHOLD * 0.1
        return int(score)
        

    '''
    @Params:
    t_pos: timestep of the point.
    t_crr: timestep of current agent.
    '''
    def anytime_func(self, cost):
        return cost // 2

def main():
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

    env = Environment(dimension, agents, obstacles)

    # Searching
    cbs = CBS(env)
    print('Start searching ...')
    solution = cbs.search()

    # time_end = time.time()
    # print('Running time: ', time_end - time_start)
    if not solution:
        print(" Solution not found" ) 
        return

    # Write to output file
    print('Start writing ...')
    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    print('Finish writing ...')

    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)  
    
    # My part (YY)
    with open('dynamic_obstacle_pth.yaml', 'r') as d_obstacles_file:
        try:
            DYNAMIC_OBSTACLES = yaml.load(d_obstacles_file, Loader=yaml.FullLoader)['schedule']
        except yaml.YAMLError as exc:
            print(exc)

    
    total_span = max(([len(path) for path in solution.values()])) # total cost of timestep
    jump_time = 0
    for timestep in range(total_span):
    # for timestep in range(3):
        print('>> Timestep ' + str(timestep))
        if jump_time > 0:
            jump_time -= 1
            continue
        for agent in agents:
            agent_name = agent['name']
            A = Agent(agent_name, solution)
            points = A.detect(timestep, DYNAMIC_OBSTACLES)

            if not points:
                continue
            constraint_dict, anytime_limitation = A.find_constraint_list()
            jump_time = anytime_limitation

            def g(x):
                # print('x.time: ' + str(x.time))
                # print('x.location: ' + str(x.location))
                # print('(timestep + anytime_limitation)' + str((timestep + anytime_limitation)))
                x.time -= (timestep + anytime_limitation)
                # print('x.time: ' + str(x.time))
                return x
            for agent in constraint_dict.keys():
                # print(agent)
                constraint_dict[agent].vertex_constraints = set(map(g, constraint_dict[agent].vertex_constraints))
            for i in constraint_dict['agent4'].vertex_constraints:
                print('@@@@@: ' + str(i))
            # print('=====')
            # print('time: ' + str(timestep + anytime_limitation))
            # print(constraint_dict['agent4'])
            # print('=====')

            # change agents start time
            agents_cp = deepcopy(agents)
            for agent in agents_cp:
                if timestep + anytime_limitation >= len(solution[agent['name']]):
                    temp = solution[agent['name']][len(solution[agent['name']]) - 1]
                else:
                    temp = solution[agent['name']][timestep + anytime_limitation]
                agent['start'] = [temp['x'], temp['y']]
            # print('******')
            # print(agents_cp)
            # print('******')
            env = Environment(dimension, agents_cp, obstacles)
            # Searching
            cbs = CBS(env, is_add_constraint=True, decentralized_constraint_list=constraint_dict)
            print('Start searching ...')
            solution = cbs.search()

            # Update output file from time: timestep + anytime_limitation.
            # Combine previous solution with the new one

            # Get previous solution
            with open(args.output, 'r') as output_yaml:
                try:
                    output_pre = yaml.load(output_yaml, Loader=yaml.FullLoader)
                except yaml.YAMLError as exc:
                    print(exc)
            solution_pre = output_pre['schedule']

            # util: map function
            def f(x):
                x['t'] += timestep + anytime_limitation
                return x

            # combine previous solution [:timestep + anytime_limitation] and new solution
            for agent in solution.keys():
                solution[agent] = solution_pre[agent][:timestep + anytime_limitation] + (list(map(f, solution[agent]))) 
            output = {}
            output["schedule"] = solution
            output["cost"] = env.compute_solution_cost(solution)
            
            # write solution to file
            with open('output_temp.yaml', 'w') as output_yaml:
                yaml.safe_dump(output, output_yaml) 


        
    
if __name__ == "__main__":
    main()