from pulp import *
import os
import numpy as np
from go_to_points import *


class Task:
    def __init__(self):
        self.tid = ""
        self.origin = [0, 0]
        self.destination = [0, 0]
        self.deadline = 0
        self.workload = 5


class Agent:
    def __init__(self):
        self.aid = ""
        self.location = [0, 0]
        self.tasks = ["move"]


class TaskAllocation:
    def __init__(self):
        self.agents = []
        self.tasks = []
        pass

    def add_agent(self, agt):
        self.agents.append(agt)

    def add_task(self, tsk):
        self.tasks.append(tsk)

    def remove_task(self, tsk):
        self.tasks.remove(tsk)

    def remove_agent(self, agt):
        self.agents.remove(agt)

    def calc_cost(self, agt, tak):
        x = np.array(agt.location)
        y = np.array(tak.origin)
        z = np.array(tak.destination)
        res = 0.0
        res += np.linalg.norm(x - y)
        res += np.linalg.norm(y - z)
        return res

    def assign(self, agents, tasks):
        num_agent = len(agents)
        num_task = len(tasks)
        agent_inds = range(0, num_agent)
        task_inds = range(0, num_task)
        assignment = LpVariable.dicts("assignment", (agent_inds, task_inds), cat='Binary')
        prob = LpProblem("task allocation problem")

        for a in agent_inds:
            prob += lpSum([assignment[a][t] for t in
                           task_inds]) <= num_task / num_agent + 1  # each agent can only assign one task

        for t in task_inds:
            prob += lpSum([assignment[a][t] for a in agent_inds]) == 1  # each task must be assigned to one agent

        cost = [[1] * num_task for i in range(num_agent)]  # cost[a][t] cost of agent a assigning task t
        for a in agent_inds:
            for t in task_inds:
                cost[a][t] = self.calc_cost(agents[agent_inds[a]], tasks[task_inds[t]])

        prob += lpSum([assignment[a][t] * cost[a][t] for a in agent_inds for t in task_inds])

        prob.solve()
        tmp = [v.varValue.real for v in prob.variables()]
        tmp = np.reshape(tmp, (len(agent_inds), len(task_inds)))
        res = []
        for a in agent_inds:
            for t in task_inds:
                if tmp[a][t] == 1:
                    res.append((agents[agent_inds[a]], tasks[task_inds[t]]))

        return res


if __name__ == "__main__":
    nav = GoToPoints()
    action_list = {}
    now_status = {}
    task_allocation = TaskAllocation()
    tmp_start_x = np.array([])
    tmp_start_y = np.array([])
    tmp_end_y = np.array([])
    start_x = np.linspace(-0.4, 1.3, num=5)
    start_y = np.linspace(0, 0, num=5)
    end_y = np.linspace(1, 1, num=5)
    for i in range(len(start_y)):
        temp = np.linspace(start_y[i], start_y[i], num=5)
        end_tmp = np.linspace(end_y[i], end_y[i], num=5)
        tmp_start_x = np.r_[tmp_start_x, start_x]
        tmp_start_y = np.r_[tmp_start_y, temp]
        tmp_end_y = np.r_[tmp_end_y, end_tmp]
    start = np.c_[tmp_start_x, tmp_start_y]
    end = np.c_[tmp_start_x, tmp_end_y]
    if nav.debug == 1:
        x = nav.facility.get_poses()
        nav.facility.step()
    else:
        x = nav.facility.get_real_position()
    for i in range(len(start)):
        task = Task()
        task.origin = start[i]
        task.destination = end[i]
        task_allocation.add_task(task)
    location = x[0:2, :].T
    for i in range(nav.N):
        action_list[i] = []
        now_status[i] = 0
        agent = Agent()
        agent.aid = str(i)
        agent.location = location[i]
        task_allocation.add_agent(agent)
    task_list = task_allocation.assign(task_allocation.agents, task_allocation.tasks)
    for i in range(len(task_list)):
        robot_id = int(task_list[i][0].aid)
        action_list[robot_id].append(task_list[i][1])
    goal_position = x.copy()
    while True:
        if nav.debug == 1:
            x = nav.facility.get_debug_position()
        else:
            x = nav.facility.get_real_position()
        last_position = goal_position.copy()
        for i in range(nav.N):
            if action_list[i]:
                if np.linalg.norm(x[0:2, i] - goal_position[0:2, i]) < 0.05 and now_status[i] == 0:
                    now_status[i] = 1
                    goal_position[0, i] = action_list[i][-1].origin[0]
                    goal_position[1, i] = action_list[i][-1].origin[1]
                if np.linalg.norm(x[0:2, i] - goal_position[0:2, i]) < 0.05 and now_status[i] == 1:
                    now_status[i] = 0
                    goal_position[0, i] = action_list[i][-1].destination[0]
                    goal_position[1, i] = action_list[i][-1].destination[1]
                    action_list[i].pop()
        if ~(last_position == goal_position).all():
            start_points = []
            end_points = []
            for i in range(nav.N):
                start_points.append(x[0:2, i].tolist())
                end_points.append(goal_position[0:2, i].tolist())
            start_points = [y for x in start_points for y in x]
            end_points = [y for x in end_points for y in x]
            nav.navigator.reset(start_points, end_points)
            path = nav.navigator.get_path()
            print(path)
        print(goal_position)
        print(x)
        nav.head_to_rvo(goal_position)
