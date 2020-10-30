import numpy as np
from copy import deepcopy
import random

from agent import Agent
from node import Node
from RRAStar import RRAStar
from utils import get_neighbors_forward, calculate_distance, DistanceMethod
from visualization import Graph
import time

class WHCA(object):
    def __init__(self, starts, goals, cost_map, window, agent_radius=0.2, visualization=False):
        self.original_cost_map = cost_map
        self.cost_map = []

        self.agent_solver = []  # (agent, solver)

        self.window = window
        self.agent_radius = agent_radius

        self.reset_cost_map(self.original_cost_map)

        self.visualization = visualization
        if self.visualization:
            self.graph = Graph(shape=self.original_cost_map.shape[0])

        if len(starts) != len(goals):
            raise Exception("starts or goals should have same number of elements as number of agent")

        for i in range(len(starts)):
            agent = Agent(Node(np.array(starts[i])), Node(np.array(goals[i])))
            rra_star = RRAStar(agent, self.original_cost_map)
            rra_star.run_initial()
            self.agent_solver.append((agent, rra_star))

    def reset_cost_map(self, cost_map):
        self.cost_map = []
        for w in range(self.window):
            self.cost_map.append(deepcopy(cost_map))

    def _judge_agent(self, i):
        if len(self.agent_solver[i][0].trajectory) < 5:
            return
        pose1 = self.agent_solver[i][0].trajectory[-1].position
        pose2 = self.agent_solver[i][0].trajectory[-2].position
        pose3 = self.agent_solver[i][0].trajectory[-3].position
        pose4 = self.agent_solver[i][0].trajectory[-4].position

        a = pose1[0] == pose3[0] and pose1[1] == pose3[1]
        b = pose2[0] == pose4[0] and pose2[1] == pose4[1]
        c = pose2[0] != pose1[0] or pose2[1] != pose1[1]
        if a and b and c:
            agent = Agent(Node(np.array(pose1)), self.agent_solver[i][0].goal)
            agent.trajectory = deepcopy(self.agent_solver[i][0].trajectory)
            rra_star = RRAStar(agent, self.original_cost_map)
            rra_star.run_initial()
            self.agent_solver[i] = (agent, rra_star)

    def _choose_best_neighbor(self, i, w, current_node):
        neighbors = get_neighbors_forward(self.cost_map[w], current_node)

        # neighbors = []
        # last_positions = []
        # for win in range(self.window - 1):
        #     if len(self.agent_solver[i][0].trajectory) > win + 2:
        #         last_positions.append(deepcopy(self.agent_solver[i][0].trajectory[-win-2].position))
        # for nei in before_selected:
        #     tag = True
        #     if len(last_positions) == 0:
        #         tag = False
        #     for pos in last_positions:
        #         if nei.position[0] != pos[0] or nei.position[1] != pos[1]:
        #             tag = False
        #     if tag:
        #         neighbors.append(nei)

        # self.solvers[i].update_cost_map(self.cost_map[w])
        if len(neighbors) == 0:
            return None
        node = neighbors[0]
        check_if_closed = False
        for test_tuple in self.agent_solver[i][0].closed_set:
            if test_tuple[1] == node:
                check_if_closed = True
                node.f_score = current_node.f_score + calculate_distance(node.position, current_node.position,
                                                                         DistanceMethod.StraightLine) + \
                               calculate_distance(node.position, current_node.position, DistanceMethod.Manhattan)
        if not check_if_closed:
            self.agent_solver[i][1].run_resume(node)
            node = self.agent_solver[i][0].experienced_nodes[tuple(node.position)][0]

        for neighbor in neighbors:
            # self.solvers[i].update_cost_map(self.cost_map[w])
            if self.agent_solver[i][1].run_resume(neighbor):
                temp_node = self.agent_solver[i][0].experienced_nodes[tuple(neighbor.position)][0]
                if temp_node.f_score < node.f_score:
                    node = temp_node
            else:
                temp_node = self.agent_solver[i][0].experienced_nodes[tuple(neighbor.position)][0]
                temp_node.f_score = current_node.f_score + calculate_distance(temp_node.position, current_node.position,
                                                                              DistanceMethod.StraightLine) + \
                                    calculate_distance(node.position, current_node.position, DistanceMethod.Manhattan)
                if temp_node.f_score < node.f_score:
                    node = temp_node

        return node

    def track_back_node(self, i, current_node, w):
        for temp_w in range(self.window):
            if temp_w >= w:
                self.cost_map[temp_w].check_self_collision(current_node.position)

        # TODO: it should be in closed set, but here I just use experienced set
        if tuple(current_node.position) in self.agent_solver[i][0].experienced_nodes:
            node = self.agent_solver[i][0].experienced_nodes[tuple(current_node.position)][1]
            if self.cost_map[w].check_collision(node.position):
                node = self._choose_best_neighbor(i, w, current_node)
        else:
            node = self._choose_best_neighbor(i, w, current_node)

        if node is None:
            return False

        self.agent_solver[i][0].trajectory.append(node)
        # self.agent_solver[i][0].last_position = node.position

        self.cost_map[w].add_obstacle([node.position[0], node.position[1]])
        return True

    def step(self, curren_cost_map):
        self.reset_cost_map(curren_cost_map)
        for i in range(len(self.agent_solver)):
            # print("in agent: {}".format(i))
            for w in range(self.window):
                # print("in window: {}".format(w))
                current_node = self.agent_solver[i][0].trajectory[-1]
                tag = self.track_back_node(i, current_node, w)

    def run(self, start_time, timestep):
        success = True
        for i in range(len(self.agent_solver)):
            if time.time() - start_time >= timestep:
                return 'TIME'
            success &= self.agent_solver[i][0].trajectory[-1] == self.agent_solver[i][0].goal

        while not success:
            if time.time() - start_time >= timestep:
                return 'TIME'
            # print("before: {}".format(self.agents))
            # random.shuffle(self.agent_solver)

            # print("after: {}".format(self.agents))
            cost_map = deepcopy(self.original_cost_map)
            for i in range(len(self.agent_solver)):
                if time.time() - start_time >= timestep:
                    return 'TIME'
                agent = self.agent_solver[i]
                cost_map.add_obstacle([agent[0].trajectory[-1].position[0], agent[0].trajectory[-1].position[1]])

                self._judge_agent(i)

            # self.step(self.cost_map[-1])
            self.step(cost_map)

            success = True

            for i in range(len(self.agent_solver)):
                success &= self.agent_solver[i][0].trajectory[-1] == self.agent_solver[i][0].goal

            if self.visualization:
                trajectories = []
                for i in range(len(self.agent_solver)):
                    print("{},{}".format(self.agent_solver[i][0].trajectory[-5].position[0],
                                         self.agent_solver[i][0].trajectory[-5].position[1]), end=" / ")
                    trajectories.append([ [self.agent_solver[i][0].trajectory[-5].position[0],
                                           self.agent_solver[i][0].trajectory[-5].position[1]] ])
                print()
                for i in range(len(self.agent_solver)):
                    print("{},{}".format(self.agent_solver[i][0].trajectory[-4].position[0],
                                         self.agent_solver[i][0].trajectory[-4].position[1]), end=" / ")
                    trajectories[i].append([self.agent_solver[i][0].trajectory[-4].position[0],
                                         self.agent_solver[i][0].trajectory[-4].position[1]])
                print()
                for i in range(len(self.agent_solver)):
                    print("{},{}".format(self.agent_solver[i][0].trajectory[-3].position[0],
                                         self.agent_solver[i][0].trajectory[-3].position[1]), end=" / ")
                    trajectories[i].append([self.agent_solver[i][0].trajectory[-3].position[0],
                                         self.agent_solver[i][0].trajectory[-3].position[1]])
                print()
                for i in range(len(self.agent_solver)):
                    print("{},{}".format(self.agent_solver[i][0].trajectory[-2].position[0],
                                         self.agent_solver[i][0].trajectory[-2].position[1]), end=" / ")
                    trajectories[i].append([self.agent_solver[i][0].trajectory[-2].position[0],
                                            self.agent_solver[i][0].trajectory[-2].position[1]])
                print()

                for i in range(len(self.agent_solver)):
                    print("{},{}".format(self.agent_solver[i][0].trajectory[-1].position[0],
                                         self.agent_solver[i][0].trajectory[-1].position[1]), end=" / ")
                    trajectories[i].append([self.agent_solver[i][0].trajectory[-1].position[0],
                                            self.agent_solver[i][0].trajectory[-1].position[1]])

                starts = []
                goals = []
                current_positions = []
                for i in self.agent_solver:
                    if time.time() - start_time >= timestep:
                        return 'TIME'
                    starts.append([i[0].start.position[0], i[0].start.position[1]])
                    goals.append([i[0].goal.position[0], i[0].goal.position[1]])
                    current_positions.append([i[0].trajectory[-1].position[0],i[0].trajectory[-1].position[1]])
                # self.graph.show_map(self.original_cost_map.obstacles, starts, goals,  trajectories, tag=True)
                self.graph.show_map(self.original_cost_map.obstacles, current_positions, [],  trajectories, tag=True)
                print()

        return self.agent_solver
