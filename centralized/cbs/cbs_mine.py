"""

Python implementation of Conflict-based search

author: Ashwin Bose (@atb033)

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

class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))

class Conflict(object):
    VERTEX = 1
    EDGE = 2    
    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''
    
        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')' 

class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))        
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')' 

class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')' 

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])

class Environment(object):
    def __init__(self, dimension, agents, obstacles, obstacles_d=[], w_l=1.03, alpha1=1, alpha2=1):
        print(obstacles_d)
        self.dimension = dimension
        self.obstacles = obstacles
        self.obstacles_d = obstacles_d

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

        # AFS part
        self.w_l = w_l # Bound of lower level
        self.alpha1 = alpha1 # coefficient of conflict number
        self.alpha2 = alpha2 # coefficient of dynamic obstacles distance

    def get_neighbors(self, state):
        neighbors = []
        
        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors

    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            # First kind of conflict
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            # First kind of conflict
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result                
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint
        
        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)
        
            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles \
            and (state.location.x, state.location.y) not in self.obstacles_d 

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)

    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))
            
            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})

    def compute_solution(self):
        solution = {}
        start_time = time.time()
        for agent in self.agent_dict.keys():
            # Get the constraint of this agent
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            # Use A* to search ang get the solution for this agent
            local_solution = self.a_star.search(agent, self.w_l, self.alpha1, self.alpha2, solution, self.obstacles_d)
            if not local_solution:
                return False
            # update the solution with this local solution of the agent
            solution.update({agent:local_solution})
        end_time = time.time()
        print('Use time: ' + str(end_time - start_time))
        # output = {}
        # output["schedule"] = self.generate_plan(solution)
        # output["cost"] = 1
        
        # # write solution to file
        # with open('output_debug.yaml', 'w') as output_yaml:
        #     yaml.safe_dump(output, output_yaml) 
        return solution

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])

class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __lt__(self, other):
        return self.cost < other.cost

'''
@limited_time: the time limitation in anytime algorithm
'''
class CBS(object):
    def __init__(self, environment, limited_time=15, is_add_constraint=False, decentralized_constraint_list=None):
        self.env = environment 
        self.is_add_constraint = is_add_constraint
        self.decentralized_constraint_list = decentralized_constraint_list
        self.open_set = set()

        # Anytime things
        self.focal_list = []
        self.limited_time = limited_time
        self.gamma = 0.9 # Decay of bound
        self.w_h = 1.1 / self.gamma # Bound of high level

    def updateFocalBound(self, bound):
        for i in range(len(self.focal_list)):
            node = self.focal_list[i]
            if node.cost > bound:
                self.focal_list.pop(i)

    def updateLowerBound(self, old_bound, new_bound):
        for new_node in self.open_set:
            if (new_node.cost > old_bound) and (new_node.cost < new_bound):
                for i in range(len(self.focal_list)):
                    node = self.focal_list[i]
                    if len(new_node.constraint_dict) <= len(node.constraint_dict):
                        self.focal_list.insert(i, new_node)
                        break
                    if i == len(self.focal_list) - 1:
                        self.focal_list.append(new_node)

    def getNextBound(self):
        return self.w_h * self.gamma

    def search(self):
        start = HighLevelNode()

        # 1. Node.constraint_dict initialization
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()

        # 2. Node.solution initialization
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}

        # 3. Node.cost initialization
        start.cost = self.env.compute_solution_cost(start.solution)

        # trivial things preparation
        cnt = 0 # counter

        # OPEN & FOCAL initialization
        self.open_set |= {start}
        self.focal_list.append(start)

        # TODO: Add the time limitation iteration
        
        while self.focal_list:
            # Count number of iterations and print it
            cnt += 1
            if cnt % 1000 == 0  and cnt > 1:
                print('iteration: ', cnt)

            # Tighten the bound
            w_h = self.getNextBound()

            # Update Focal list
            f_min = min(self.open_set).cost
            self.updateFocalBound(w_h * f_min) # Here, f_min is the f(head(OPEN))

            # Order Focal list if not efficiently reusable (Here, it is. So do nothing)
            
            # Get the expanded state from FOCAL
            P = self.focal_list[0]

            # Delete the node from both OPEN & FOCAL
            self.open_set -= {P}
            self.focal_list.pop(0)

            # Update environment
            self.env.constraint_dict = P.constraint_dict

            # Get the first conflict from all agents
            conflict_dict = self.env.get_first_conflict(P.solution)

            # If there's no conflict for all paths of agents, solution if found!
            if not conflict_dict:
                if (self.is_add_constraint): # 也就是有decentralized的更新 | USELESS NOW, self.is_add_constraint === FALSE
                    """
                    做三件事情：
                    1. 把更新过来的constraint加入到新node的constraint list里面，且继承父亲node的constraint list
                    2. A*计算新的solution
                    3. open中只保留这个新的node

                    待改进：
                    1. 之前的open可以保存，有decentralized的update的时候可以直接reuse
                    """
                    print('Old solution found. Start add constraint list.')
                    self.is_add_constraint = False
                    new_node = deepcopy(P)
                    pre_constraint_dict = P.constraint_dict[agent]
                    new_node.constraint_dict[agent].add_constraint(pre_constraint_dict)
                    for agent in self.decentralized_constraint_list.keys():
                        new_node.constraint_dict[agent].add_constraint(self.decentralized_constraint_list[agent]) 
                        self.env.constraint_dict = new_node.constraint_dict
                    new_node.solution = self.env.compute_solution()
                    new_node.cost = self.env.compute_solution_cost(new_node.solution)
                    self.open_set = {new_node}
                    continue
                else:
                    print("solution found")
                    print('Iteration number: ', cnt)
                    output = {}
                    output["schedule"] = self.generate_plan(P.solution)
                    output["cost"] = 1999
                    
                    # write solution to file
                    with open('output_debug.yaml', 'w') as output_yaml:
                        yaml.safe_dump(output, output_yaml) 

                    return self.generate_plan(P.solution)

            # Have conflict, generate constraint dict
            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            # Generate 2 son nodes
            for agent in constraint_dict.keys():
                # 1. Extend constraint dict from father node
                new_node = deepcopy(P)

                # 2. Add new constrant to constraint dict according to which agent you choose
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent]) 
                
                # Update environment constraint
                self.env.constraint_dict = new_node.constraint_dict # A*就是根据self.env.constraint_dict来得到solution的

                # 3. Update solution with new constraints added
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue

                # 4. Add cost
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # Add the son node to open set
                self.open_set |= {new_node}

                # If it's conform to the standard(cost <= w * f_min), insert it to the focal list by the conflict number
                if new_node.cost <= w_h * f_min:
                    for i in range(len(self.focal_list)):
                        node = self.focal_list[i]
                        if len(new_node.constraint_dict) <= len(node.constraint_dict):
                            self.focal_list.insert(i, new_node)
                            break
                        if i == len(self.focal_list) - 1:
                            self.focal_list.append(new_node)
            
            # Update the focal list because lower bound of open set is changed, sth may come into the open set~
            f_min_new = min(self.open_set).cost
            if (len(self.open_set) != 0) and (self.w_h * f_min < self.w_h * f_min_new):
                self.updateLowerBound(f_min, f_min_new)

        return {}

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan


def main():
    time_start = time.time()

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

    time_end = time.time()
    print('Running time: ', time_end - time_start)
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
    # print(solution)
    
if __name__ == "__main__":
    main()
