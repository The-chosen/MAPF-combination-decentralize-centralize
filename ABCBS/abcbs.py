"""

Description: Python implementation of Anytime bounded conflict-based search (Type 1)

Author: Yue Yang (yuey9923@gmail.com)

"""

import argparse
import yaml
from itertools import combinations
from copy import deepcopy
import time
import random
import logging
from Environment import Environment, Constraints

class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __lt__(self, other):
        return self.cost < other.cost

class ABCBS(object):
    def __init__(self, environment, logger, limited_time = 30, is_anytime = False, w_h = 1.1, w_l = 1.1, gamma = 0.985):
        self.env = environment
        self.is_anytime = is_anytime
        self.logger = logger
        self.open_set = set()

        # Anytime things
        self.focal_list = []
        self.limited_time = limited_time
        self.gamma = gamma # Decay of bound
        self.w_h = w_h / self.gamma # Bound of high level
        self.w_l = w_l

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
                        break
                if len(self.focal_list) == 0:
                    self.focal_list.append(new_node)

    def getNextBound(self):
        return_thing = self.w_h * self.gamma
        if return_thing < 1.001:
            self.logger.info("w_h < 1.001. Have been most optimal")
            return "OPTIMAL"
        return return_thing

    def open_focal_initialization(self, solutions):
        # Construct start node in high level
        start = HighLevelNode()

        # 1. Node.constraint_dict initialization
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()

        # 2. Node.solution initialization
        start.solution = self.env.compute_solution(self.start_time, self.limited_time)
        # Time exceeded, go outside ~
        if start.solution == "TIME":
            if len(solutions) == 0:
                self.logger.warning("Time exeeded. No solution found.")
                return "OVER", {}
            else:
                return "OVER", solutions[len(solutions) - 1]

        # if start.solution == "OCCUPY":
        #     continue

        if not start.solution:
            return "OVER", {}

        # 3. Node.cost initialization
        start.cost = self.env.compute_solution_cost(start.solution)


        # OPEN & FOCAL initialization
        self.open_set |= {start}
        self.focal_list.append(start)

        return None, None


    def search(self, N=0, agent_num=0, obstacle_prob=0):
        # Calc consumed time for the first success
        is_first_succ = True
        succ_start_time = time.time()

        # Find how many solutions
        num_sol = 1

        # All found solutions
        solutions = []

        # Time limitation iteration
        self.start_time = time.time()

        # Initialize OPEN & FOCAL
        msg, return_data = self.open_focal_initialization(solutions)

        if msg == "OVER":
            return return_data

        # Keep running if anytime limitation is not over
        while True:
            # If there's no node, initialize again
            if len(self.open_set) == 0 or len(self.focal_list) == 0:
                self.logger.info("One new iteration for a new Solution")
                msg, return_data = self.open_focal_initialization(solutions)

            if msg == "OVER":
                return return_data

            # Tighten the bound
            w_h = self.getNextBound()
            if w_h == "OPTIMAL":
                if len(solutions) == 0:
                    return {}
                else:
                    self.logger.info("Optimal solution found.")
                    return solutions[len(solutions) - 1]


            # Update Focal list
            f_min = min(self.open_set).cost
            self.updateFocalBound(w_h * f_min) # Here, f_min is the f(head(OPEN))

            # Has found one solution?
            has_found_sol = False

            while self.focal_list:
                self.logger.debug("Length of focal list: ", len(self.focal_list))
                
                f_min = min(self.open_set).cost

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

                self.logger.debug("Conflict dict: " + str(conflict_dict))

                # If there's no conflict for all paths of agents, solution if found!
                if not conflict_dict:
                    self.logger.info(str(num_sol) + " solution(s) are found.")
                    self.logger.info("Cost of success nude: " + str(P.cost))
                    
                    if is_first_succ:
                        is_first_succ = not is_first_succ
                        succ_end_time = time.time()
                        self.logger.info("Success! Consumed time: " + str(succ_end_time - succ_start_time))

                        # If not anytime, return the first solution immediately, else continue compute one new solution
                        if not self.is_anytime:
                            return self.generate_plan(P.solution)

                    num_sol += 1
                    has_found_sol = not has_found_sol
                    solutions.append(self.generate_plan(P.solution))
                    self.open_set = set()
                    self.focal_list = []

                if has_found_sol:
                    break

                # Have conflict, generate constraint dict
                constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)


                # Generate 2 son nodes
                for agent in constraint_dict.keys():
                    self.logger.debug("Solve conflict from: " + str(agent))

                    # 1. Extend constraint dict from father node
                    new_node = deepcopy(P)

                    # 2. Add new constrant to constraint dict according to which agent you choose
                    new_node.constraint_dict[agent].add_constraint(constraint_dict[agent]) 
                    
                    # Update environment constraint
                    self.env.constraint_dict = new_node.constraint_dict # A*就是根据self.env.constraint_dict来得到solution的

                    # 3. Update solution with new constraints added. Now we have the third param (agent), 
                    #       it can make the compute_solution only compute path for this agent
                    new_node.solution = self.env.compute_solution(self.start_time, self.limited_time, agent, deepcopy(P.solution))

                    if new_node.solution == "OCCUPY":
                        continue

                    if new_node.solution == "TIME":
                        if len(solutions) == 0:
                            self.logger.warning("Time exceeded. No solution found.")
                            return {}
                        else:
                            return solutions[len(solutions) - 1]

                    if not new_node.solution:
                        continue

                    # 4. Add cost
                    new_node.cost = self.env.compute_solution_cost(new_node.solution)

                    # Add the son node to open set
                    self.open_set |= {new_node}

                    # If it's conform to the standard(cost <= w * f_min), insert it to the focal list by the conflict number
                    self.logger.debug("New_node.cost: " + str(new_node.cost))
                    self.logger.debug("w_h * f_min: " + str(w_h * f_min))
                    
                    if new_node.cost <= w_h * f_min:
                        for i in range(len(self.focal_list)):
                            node = self.focal_list[i]
                            if len(new_node.constraint_dict) <= len(node.constraint_dict):
                                self.focal_list.insert(i, new_node)

                                break
                            if i == len(self.focal_list) - 1:
                                self.focal_list.append(new_node)
                                break
                        if len(self.focal_list) == 0:
                            self.focal_list.append(new_node)
                
                # Update the focal list because lower bound of open set is changed, sth may come into the open set~
                if len(self.open_set) != 0:
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

def print_parser(parser, opt):
    message = ''
    message += '\n----------------- Options ---------------\n'
    for k, v in sorted(vars(opt).items()):
        comment = ''
        default = parser.get_default(k)
        if v != default:
            comment = '\t[default: %s]' % str(default)
        message += '{:>25}: {:<30}{}\n'.format(str(k), str(v), comment)
    message += '----------------- End -------------------\n'
    print(message)

def main():
    # Parser initialization
    parser = argparse.ArgumentParser(description="Anytime BCBS")
    parser.add_argument("--input", type=str, default="input.yaml", help="input file containing map and obstacles")
    parser.add_argument("--output", type=str, default="output.yaml", help="output file path")
    parser.add_argument("--wh", type=float, default=1.1, help="omega value of high level")
    parser.add_argument("--wl", type=float, default=1.1, help="omega value of low level")
    parser.add_argument("--gamma", type=float, default=0.985, help="bound decay ratio")
    parser.add_argument("--is-anytime", type=bool, default=False, help="use anytime or not")
    parser.add_argument("--anytime-limitation", type=int, default=30, help="anytime limitation")
    parser.add_argument("--is-logger-file", type=bool, default=False, help="output file path")
    parser.add_argument("--logger-file-path", type=str, default="ABCBS_logger.log", help="log file path")
    parser.add_argument("--logger-level", type=str, default="INFO", \
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"], help="logging level")
    args = parser.parse_args()
    
    # Logger initialization
    if args.logger_file:
        logging.basicConfig(filename=args.logger_file_path, filemode='w', level=args.logger_level)
    else:
        logging.basicConfig(filemode='w', level=args.logger_level)
    logger = logging.getLogger('ABCBS_logger')
    logger.root.setLevel(args.logger_level)

    logger.info(">>>>>>>>>>>>>>>>>>>>>>> ABCBS START <<<<<<<<<<<<<<<<<<<<<<<<<<")
    print_parser(parser, args)

    # Read information of agents, static obstacles from input file
    logger.info("Read inputs ... ")
    with open(args.input, 'r') as input_file:
        try:
            input_info = yaml.load(input_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            logger.error(exc)

    # Start and goal positions of different agents. 
    # e.g. [{'start': [1, 2], 'goal': [1, 2], 'name': 'agent0'}, {'start': [1, 3], 'goal': [1, 5], 'name': 'agent1'}]
    agents = input_info['agents'] 

    # Dimension of the map. 
    # e.g. [32, 32], [64, 64], etc.
    dimension = input_info["map"]["dimensions"]

    # Positions (tuple) of obstacles. 
    # e.g. (0, 3) 
    obstacles = input_info["map"]["obstacles"] 

    # Start to search
    env = Environment(dimension, agents, obstacles, logger, w_l = args.wl)
    cbs = ABCBS(env, logger, args.anytime_limitation, args.is_anytime, args.wh, args.wl, args.gamma)
    logger.info("Start to search ...")
    solution = cbs.search()
    logger.info("Searching end!")
    if not solution:
        logger.warning("Solution NOT found!")
        logger.info(">>>>>>>>>>>>>>>>>>>>>>> ABCBS END <<<<<<<<<<<<<<<<<<<<<<<<<<")
        return
    else:
        logger.info("Solution found!")

    output = {}
    output['schedule'] = solution

    # Write solution to output file
    logger.info("Write solution to output file ... ")
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

    logger.info(">>>>>>>>>>>>>>>>>>>>>>> ABCBS END <<<<<<<<<<<<<<<<<<<<<<<<<<")

if __name__ == "__main__":
    main()