"""

AStar search

author: Ashwin Bose (@atb033)

"""

class AStar():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name, w_l, pre_solution):
        """
        low level search 
        """
        # print('agent_name1: ' + str(agent_name))
        initial_state = self.agent_dict[agent_name]["start"]

        step_cost = 1
        
        closed_set = set()
        open_set = {initial_state}


        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0


        f_score = {} 

        # initial_state is the coordinate
        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)

        # Focal part
        focal_set = set()

        timestep = 0

        while open_set:
            # open_item is the coordinate
            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current_idx = min(temp_dict, key=temp_dict.get)
            # print('temp_dict: ' + str(len(temp_dict)))

            # Focal part
            # f1 part: get focal list
            focal_dict = {}
            f_min = temp_dict[current_idx]
            for open_item in open_set:
                f_item = f_score.setdefault(open_item, float("inf"))
                if f_item <= f_min * w_l:
                    focal_dict[open_item] = f_item

            # print('focal_dict: ' + str(len(focal_dict)))
            # f2 part
            current = None
            if len(pre_solution.keys()) == 0:
                current = current_idx
            else:
                focal_cost_dict = {focal_item: 0 for focal_item in focal_dict.keys()} # dict. key: (t, x, y) value: number of conflict
                for agent_name_ in pre_solution.keys():
                    if timestep >= len(pre_solution[agent_name_]):
                        break
                    conflict_pos = pre_solution[agent_name_][timestep]
                    focal_cost_dict = {focal_item: focal_cost_dict[focal_item] + 1 \
                        if focal_item == conflict_pos else focal_cost_dict[focal_item] for focal_item in focal_dict.keys()}
                min_focal_cost_idx = min(focal_cost_dict, key=focal_cost_dict.get)
                min_focal_cost_dict = {focal_cost: focal_dict[focal_cost] \
                    for focal_cost in focal_cost_dict.keys() if focal_cost_dict[focal_cost] == focal_cost_dict[min_focal_cost_idx]}
                current = min(min_focal_cost_dict, key=min_focal_cost_dict.get)
            # print(current in temp_dict.keys())
            
            if self.is_at_goal(current, agent_name):
                return self.reconstruct_path(came_from, current)


            timestep += 1

            open_set -= {current}
            closed_set |= {current}

            # the get_neighbors() function will return all valid directions to go to
            # what's valid is defined by constraints list
            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)
        return False

