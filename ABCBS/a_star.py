"""

Description: Python implementation of Anytime bounded conflict-based search (Type 1)

Author: Yue Yang (yuey9923@gmail.com)

"""

import math
import time

class AStar():
    def __init__(self, env, logger):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors
        self.env = env
        self.logger = logger

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name, w_l, alpha1, alpha2, alpha3, pre_solution, obstacles_dynamic_pos_list, start_time, limited_time):
        """
        low level search 
        """

        '''
        @Explain:
            Comparison in focal list. 
            Use the created formula: alpha1 * (conflict number) - alpha2 * (distance to dynamic obstacles)
        @RETURN:
            if node1 <= node2: return True
            else: return False
        '''
        def focal_compare(node1, node2, pre_solution, obstacles_dynamic_pos_list, timestep, f_score, alpha1, alpha2, alpha3):
            # Part1: Number of conflicts with other agents
            num_conflict_node1 = 0
            num_conflict_node2 = 0
            for agent_name_ in pre_solution.keys():
                # In this timestep and for this agent, the (x, y, t) of previous solution
                if timestep >= len(pre_solution[agent_name_]):
                    break
                pre_state = pre_solution[agent_name_][timestep]
                if pre_state == node1:
                    num_conflict_node1 += 1
                if pre_state == node2:
                    num_conflict_node2 += 1

            # Part2: Sum of distance from dynamic obstacles
            distance_node1 = 0
            distance_node2 = 0
            for pos_dyn_obs in obstacles_dynamic_pos_list:
                distance_node1 += math.sqrt((pos_dyn_obs[0] - node1.location.x)**2 + (pos_dyn_obs[1] - node1.location.y)**2)
                distance_node2 += math.sqrt((pos_dyn_obs[0] - node2.location.x)**2 + (pos_dyn_obs[1] - node2.location.y)**2)

            # Part3: f score
            f_score_node1 = f_score[node1]
            f_score_node2 = f_score[node2]

            # Part4: Calculate the formula

            # Normalize
            if num_conflict_node1 == 0 and num_conflict_node2 == 0:
                num_conflict_node1 = 0
                num_conflict_node2 = 0
            else:
                num_conflict_node1 = num_conflict_node1 / (num_conflict_node1 + num_conflict_node2)
                num_conflict_node2 = num_conflict_node2 / (num_conflict_node1 + num_conflict_node2)
            
            if distance_node1 == 0 and distance_node2 == 0:
                distance_node1 = 0
                distance_node2 = 0
            else:
                distance_node1 = distance_node1 / (distance_node1 + distance_node2)
                distance_node2 = distance_node2 / (distance_node1 + distance_node2)

            # Caculation
            value_node1 = alpha1 * num_conflict_node1 - alpha2 * distance_node1 + alpha3 * f_score_node1
            value_node2 = alpha1 * num_conflict_node2 - alpha2 * distance_node2 + alpha3 * f_score_node2

            return True if value_node1 <= value_node2 else False

        # See if the goal occupied first. If it is, return a long long route~
        if agent_name in self.env.constraint_dict.keys():
            constraint_dict_ = self.env.constraint_dict[agent_name]
            for constraint in constraint_dict_.vertex_constraints:
                if (constraint.location.x == self.env.agent_dict[agent_name]["goal"].location.x) \
                    and (constraint.location.y == self.env.agent_dict[agent_name]["goal"].location.y):
                    self.logger.info("Occupy goal constraint: " + str(constraint))
                    return "OCCUPY"

        initial_state = self.agent_dict[agent_name]["start"]

        step_cost = 1 # A constant. Cost for each step (edge)
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        # initial_state is the coordinate
        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)

        # Focal part
        focal_list = [initial_state]

        timestep = 0

        while focal_list:

            # See if exceed the time limitation
            end_time = time.time()
            
            if end_time - start_time >= limited_time:
                self.logger.warning("Time exceeded! Use time: " + str(end_time - start_time) + " s.")
                return "TIME"


            # open_item is the coordinate. temp_dict.keys(): OPEN; temp_dict.values(): f
            temp_dict = {open_item: f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current_idx = min(temp_dict, key=temp_dict.get)

            # f(head(OPEN))
            f_min = temp_dict[current_idx]

            # Min node in focal list
            current = focal_list[0]

            # Remove the node from OPEN & FOCAL. Add it to CLOSE
            open_set -= {current}
            focal_list.pop(0)
            closed_set |= {current}

            # Is goal?
            if self.is_at_goal(current, agent_name):
                return self.reconstruct_path(came_from, current)

            timestep += 1

            # The get_neighbors() function will return all valid directions to go to
            # what's valid is defined by constraints list
            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                is_add = False # will the neighbor added to OPEN
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbor not in open_set:
                    is_add = True
                    open_set |= {neighbor}

                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)

                # If there's new node added to OPEN, you should consider whether it should be added to FOCAL
                if is_add:
                    if f_score[neighbor] <= w_l * f_min:
                        for i in range(len(focal_list)):
                            node = focal_list[i]
                            if focal_compare(neighbor, node, pre_solution, obstacles_dynamic_pos_list, timestep, f_score, alpha1, alpha2, alpha3):
                                focal_list.insert(i, neighbor)
                                break
                            if i == len(focal_list) - 1:
                                focal_list.append(neighbor)
                                break
                        if len(focal_list) == 0:
                            focal_list.append(neighbor)
                            
            # Update the focal list because lower bound of open set is changed, sth may come into the open set~
            temp_dict_ = {open_item: f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            if len(temp_dict_) == 0:
                return False
            current_idx_ = min(temp_dict_, key=temp_dict_.get)
            f_min_new = temp_dict_[current_idx_]
            if (len(open_set) != 0) and (f_min < f_min_new):
                # updateLowerBound part
                old_bound = w_l * f_min
                new_bound = w_l * f_min_new
                for new_node in open_set:
                    if (f_score[new_node] > old_bound) and (f_score[new_node] < new_bound):
                        for i in range(len(focal_list)):
                            node = focal_list[i]
                            if focal_compare(neighbor, node, pre_solution, obstacles_dynamic_pos_list, timestep, f_score, alpha1, alpha2, alpha3):
                                focal_list.insert(i, new_node)
                                break
                            if i == len(focal_list) - 1:
                                focal_list.append(new_node)
                                break
                        if len(focal_list) == 0:
                            focal_list.append(new_node)
                            
        self.logger.warning(agent_name + " END, it doesn't find its path :(")            
        return False

