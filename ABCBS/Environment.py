"""

Description: Python implementation of Anytime bounded conflict-based search (Type 1)

Author: Yue Yang (yuey9923@gmail.com)

"""

import sys
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy
from a_star import AStar
import time
import random
import logging

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
    def __init__(self, dimension, agents, obstacles, logger, obstacles_d = [], w_l = 1.1, alpha1 = 0.0, alpha2 = 0.0, alpha3 = 1):

        self.logger = logger

        self.dimension = dimension
        self.obstacles = obstacles
        self.obstacles_d = obstacles_d

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self, logger)

        # AFS part
        self.w_l = w_l # Bound of lower level
        self.alpha1 = alpha1 # coefficient of conflict number
        self.alpha2 = alpha2 # coefficient of dynamic obstacles distance
        self.alpha3 = alpha3 # coefficient of f score

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

            # First kind of conflict (vertex conflict)
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

            # Second kind of conflict (Edge conflict)
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
            and (state.location.x, state.location.y) not in self.obstacles

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

    def compute_solution(self, start_time, limited_time, agent_name=None, solution=None):
        self.logger.debug("Start computing solution")

        solution = {} if solution == None else solution
        start_time_ = time.time()
        if agent_name == None:
            for agent in self.agent_dict.keys():
                # Get the constraint of this agent
                self.constraints = self.constraint_dict.setdefault(agent, Constraints())
                # Use A* to search ang get the solution for this agent
                local_solution = self.a_star.search(agent, self.w_l, self.alpha1, 
                self.alpha2, self.alpha3, solution, self.obstacles_d, start_time, limited_time)
                if local_solution == 'OCCUPY':
                    return 'OCCUPY'
                if local_solution == "TIME":
                    return "TIME"
                if not local_solution:
                    return False
                # update the solution with this local solution of the agent
                solution.update({agent: local_solution})
        else:
            # Get the constraint of this agent
            self.constraints = self.constraint_dict.setdefault(agent_name, Constraints())
            # Use A* to search ang get the solution for this agent
            local_solution = self.a_star.search(agent_name, self.w_l, self.alpha1, 
            self.alpha2, self.alpha3, solution, self.obstacles_d, start_time, limited_time)
            if local_solution == 'OCCUPY':
                return 'OCCUPY'
            if local_solution == "TIME":
                return "TIME"
            if not local_solution:
                return False
            # update the solution with this local solution of the agent
            solution.update({agent_name: local_solution})
            
        end_time_ = time.time()
        self.logger.debug("Consumed time for one solution computing: " + str(end_time_ - start_time_))

        return solution

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} \
                if type(state).__name__ == 'State' else '' for state in path]
            plan[agent] = path_dict_list
        return plan

    def compute_solution_cost(self, solution):
        if type(solution).__name__ == 'str':
            return 'NO COST: ' + solution    
        return sum([len(path) for path in solution.values()])
