#!/usr/bin/env python3

from utilities.transformations import *
from utilities.graph import *
from pios_facility import PiosFacility
import numbers as np
from get_init import pios_facility_parameter
import matplotlib.pyplot as plt
from utilities.barrier_certificates import *
from utilities.misc import *
from utilities.controllers import *
import time
from pynput import keyboard
import itertools
import random
from navigationInterface import NavigationAdapter, Observation, ObservationAdapter, MotionSolutionAdapter
import sys

class GoToPoints:
    def __init__(self):
        self.pios_facility_parameter = pios_facility_parameter()
        self.method, self.N, self.debug, self.position_error, self.rotation_error = self.pios_facility_parameter.get()
        self.facility = PiosFacility(number_of_robots=self.N, show_figure=True, initial_conditions=np.array([]),
                                     sim_in_real_time=True)
        print(111)
        self.facility.robot_id_tf(self.debug)
        print(112)
        self.dxu = np.zeros((2, self.N))
        self.arrival_car = [0] * self.N
        self.add_obstacle_flag = [0] * self.N
        self.car_radio = 0.06
        self.stop_flag = 0
        if self.method == "rvo":
            print(113)
            obstacle_list = self.facility.find_obstacle()
            for i in range(1000):
                obstacle_list = self.facility.find_obstacle()
                if obstacle_list:
                    break
            print(111)
            self.navigator = NavigationAdapter(self.pios_facility_parameter.rvo_init(obstacle_list, self.N).get_dict_pointer())
            # self.navigator = NavigationAdapter("./fourstarts.json")
            self.navigator.plan("whca")
            self.observation_wrapper = ObservationAdapter()
        print("init ok")
        # Create unicycle pose controller
        self.unicycle_pose_controller = self.pios_facility_parameter.controller()
        self.si_to_uni_dyn = self.pios_facility_parameter.uni_dynamics()

        # Create barrier certificates to avoid collision
        self.uni_barrier_cert = create_unicycle_barrier_certificate()

    def model(self):
        return self.debug

    def move(self, initial_points, goal_points):
        if self.method == "bc":
            self.move_barrier_certificate(initial_points, goal_points)
        elif self.method == 'rvo':
            self.move_rvo(initial_points, goal_points)

    def move_to(self, goal_points):
        if self.debug != 1:
            initial_points = self.facility.get_real_position()
            self.move(initial_points, goal_points)
        else:
            initial_points = [[1], [1], [0]]
            self.move(initial_points, goal_points)

    def move_barrier_certificate(self, initial_points, goal_points):
        # define x initially
        if self.debug == 1:
            x = self.facility.get_poses()
            self.facility.step()
        else:
            x = self.facility.get_real_position()
        while np.size(at_pose(x, goal_points)) != self.N and self.stop_flag == 0:
            # Get poses of agents
            x = self.facility.get_poses()
            dxu = self.unicycle_pose_controller(x, goal_points)
            dxu = self.uni_barrier_cert(dxu, x)
            for i in range(self.N):
                if np.linalg.norm(x[0:2, i] - goal_points[0:2, i]) < self.position_error:
                    goal_points[0][i] = x[0][i]
                    goal_points[1][i] = x[1][i]
                    dxu[0][i] = 0.0
                    if np.linalg.norm(x[2, i] - goal_points[2, i]) < self.rotation_error:
                        dxu[1][i] = 0.0
            # Set the velocities by mapping the single-integrator inputs to unciycle inputs
            self.facility.set_velocities(np.arange(self.N), dxu)
            # Iterate the simulation
            if self.debug == 1:
                self.facility.step()
            else:
                self.facility.step_real()
                time.sleep(0.033)
                self.facility.get_real_position()
            self.facility.poses_publisher()

    def head_to_barrier_certificate(self, goal_points):
        x = self.facility.get_poses()
        dxu = self.unicycle_pose_controller(x, goal_points)
        dxu = self.uni_barrier_cert(dxu, x)
        for i in range(self.N):
            if np.linalg.norm(x[0:2, i] - goal_points[0:2, i]) < self.position_error:
                goal_points[0][i] = x[0][i]
                goal_points[1][i] = x[1][i]
                dxu[0][i] = 0.0
                if np.linalg.norm(x[2, i] - goal_points[2, i]) < self.rotation_error:
                    dxu[1][i] = 0.0
            # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        self.facility.set_velocities(np.arange(self.N), dxu)
        if self.debug == 1:
            self.facility.step()
        else:
            self.facility.step_real()
            time.sleep(0.033)
            self.facility.get_real_position()

    def move_rvo(self, initial_points, goal_points):
        # set start_points and goal_points
        start_points = []
        end_points = []
        if self.debug == 1:
            x = self.facility.get_poses()
            self.facility.step()
        else:
            x = self.facility.get_real_position()
        for i in range(self.N):
            start_points.append(x[0:2, i].tolist())
            end_points.append(goal_points[0:2, i].tolist())
        start_points = [y for x in start_points for y in x]
        end_points = [y for x in end_points for y in x]
        print(start_points)
        print(end_points)
        self.navigator.reset(start_points, end_points)

        while np.size(at_pose(x, goal_points)) != self.N and self.stop_flag == 0:
            x = self.facility.get_poses()
            current_points = []
            for i in range(self.N):
                current_points.append((x[0:2, i]).tolist())
            current_points = [y for z in current_points for y in z]
            self.observation_wrapper.set_positions(current_points)
            solution = MotionSolutionAdapter(self.navigator.step(self.observation_wrapper.get_value()))
            get_vel = solution.get_motions()
            get_vel = np.array(get_vel).reshape(-1, 2)
            # self.observation_wrapper.set_emergencies([False, False])
            count_num = 0
            for i in range(self.N):
                if self.arrival_car[i] == 1:
                    count_num += 1
            for i in range(self.N):
                if solution.get_motions():
                    dxi = np.array([[get_vel[i][0]], [get_vel[i][1]]])
                else:
                    dxi = np.zeros((2, 1))
                self.dxu[:, [i]] = self.si_to_uni_dyn(dxi, x[:, [i]])
                if np.linalg.norm(x[0:2, i] - goal_points[0:2, i]) < self.position_error:
                    self.arrival_car[i] = 1
                # if np.linalg.norm(x[0:2, i] - goal_points[0:2, i]) < self.position_error and count_num == self.N:
                if (np.linalg.norm(x[0:2, i] - goal_points[0:2, i]) - self.position_error) < self.pios_facility_parameter.position_epsilon and count_num == self.N:
                    goal_points[0][i] = x[0][i]
                    goal_points[1][i] = x[1][i]
                    self.dxu[0][i] = 0.0
                    self.dxu[1][i] = 0.4
                    if np.linalg.norm(x[2, i] - goal_points[2, i]) < self.rotation_error:
                        goal_points[2][i] = x[2][i]
                        self.dxu[1][i] = 0.0
            self.facility.set_velocities(np.arange(self.N), self.dxu)
            print("dxu:", self.dxu)
            print("arrival_car", self.arrival_car)
            if self.debug == 1:
                self.facility.step()
            else:
                self.facility.step_real()
                time.sleep(0.033)
                x = self.facility.get_real_position()
                self.facility.poses_publisher()

    def head_to_rvo(self, goal_points):
        x = self.facility.get_poses()
        current_points = []
        for i in range(self.N):
            current_points.append((x[0:2, i]).tolist())
        current_points = [y for z in current_points for y in z]
        self.observation_wrapper.set_positions(current_points)
        solution = MotionSolutionAdapter(self.navigator.step(self.observation_wrapper.get_value()))
        get_vel = solution.get_motions()
        get_vel = np.array(get_vel).reshape(-1, 2)
        count_num = 0
        for i in range(self.N):
            if self.arrival_car[i] == 1:
                count_num += 1
        for i in range(self.N):
            if solution.get_motions():
                dxi = np.array([[get_vel[i][0]], [get_vel[i][1]]])
            else:
                dxi = np.zeros((2, 1))
            self.dxu[:, [i]] = self.si_to_uni_dyn(dxi, x[:, [i]])
            if np.linalg.norm(x[0:2, i] - goal_points[0:2, i]) < self.position_error:
                self.arrival_car[i] = 1
            else:
                self.arrival_car[i] = 0
            if np.linalg.norm(x[0:2, i] - goal_points[0:2, i]) < self.position_error and count_num == self.N:
                goal_points[0][i] = x[0][i]
                goal_points[1][i] = x[1][i]
                self.dxu[0][i] = 0.0
                self.dxu[1][i] = 0.4
                if np.linalg.norm(x[2, i] - goal_points[2, i]) < self.rotation_error:
                    goal_points[2][i] = x[2][i]
                    self.dxu[1][i] = 0.0
        self.facility.set_velocities(np.arange(self.N), self.dxu)
        print("arrival_car", self.arrival_car)
        if self.debug == 1:
            self.facility.step()
        else:
            self.facility.step_real()
            time.sleep(0.033)
            x = self.facility.get_real_position()

    def backend(self):
        if self.debug == 1:
            x = self.facility.get_poses()
            self.facility.step()
        else:
            x = self.facility.get_real_position()
        goals_position = x
        while not self.stop_flag:
            t = time.time()
            pre_t = t
            change = 0
            start_points = []
            end_points = []
            last_goal = goals_position
            goals_position = self.facility.get_goals_position().copy()
            for i in range(self.N):
                if goals_position[0, i] != 0 and last_goal[0, i] != goals_position[0, i] and last_goal[1, i] != \
                        goals_position[1, i]:
                    change = 1
                if goals_position[0, i] == 0:
                    goals_position[:, i] = x[:, i]
            print(change)
            if change:
                if self.debug == 1:
                    x = self.facility.get_poses()
                    self.facility.step()
                else:
                    x = self.facility.get_real_position()
                for i in range(self.N):
                    start_points.append(x[0:2, i].tolist())
                    end_points.append(goals_position[0:2, i].tolist())
                start_points = [y for x in start_points for y in x]
                end_points = [y for x in end_points for y in x]
                print(start_points)
                print(end_points)
                self.navigator.reset(start_points, end_points)
            self.facility.poses_publisher()
            self.head_to_rvo(goals_position)
            t = time.time()
            print("---------------------------------------------")
            print("time_cost：", t - pre_t)

    def follow(self, goal_points):
        if self.method == "rvo":
            self.head_to_rvo(goal_points)
        if self.method == "bc":
            self.head_to_barrier_certificate(goal_points)

    def head_to_head(self):
        if self.debug == 1:
            initial_positions = self.facility.get_poses()
            self.facility.step()
        else:
            initial_positions = self.facility.get_real_position()
        goal_points = initial_positions.copy()
        # exchange position
        x = [i for i in range(self.N)]
        perm = np.array(list(itertools.permutations(x)))
        res = np.array([]).astype(np.int)
        for i in range(len(perm)):
            if np.prod(perm[i, :] - x) != 0:
                res = np.r_[res, perm[i, :]]
        b = int(len(res) / self.N)
        res = np.reshape(res, (b, self.N))
        index = random.sample(range(0, b), 1)
        goal_points = goal_points[:, res[index[0]]]
        self.move_to(goal_points)
        self.stop_all()

    def go_home(self):
        x = np.linspace(-0.4, -0.1, num=self.N)
        if self.N == 5:
            y = [0.5, 0.25, 0.75, 0, 1]
        else:
            y = np.linspace(0, 1, num=self.N)
        z = np.linspace(0, 0, num=self.N)
        goal_points = np.array([x, y, z])
        self.move_to(goal_points)
        self.stop_all()

    def v_formation(self):
        x = [0, -0.2, -0.2, -0.4, -0.4]
        y = [0.3, 0.15, 0.45, 0, 0.6]
        z = np.linspace(0, 0, num=self.N)
        goal_points = np.array([x, y, z])
        self.move_to(goal_points)
        self.stop_all()

    def rectangular_formation(self):
        x = [0.1, 0.3, 0.3, 0.1]
        y = [0.1, 0.1, 0.3, 0.3]
        z = np.linspace(0, 0, num=self.N)
        goal_points = np.array([x, y, z])
        self.move_to(goal_points)
        self.stop_all()

    def stop_all(self):
        for i in range(100):
            self.facility.stop()
            # print(i)
            time.sleep(0.033)

    def follow_straight_line(self):
        if self.debug == 1:
            initial_positions = self.facility.get_poses()
            self.facility.step()
        else:
            initial_positions = self.facility.get_real_position()
        goal_points = initial_positions.copy()
        for i in range(self.N):
            goal_points[0][i] += 1
            goal_points[2][i] = 0
        self.move_to(goal_points)
        self.stop_all()

    def on_press(self, key):
        if key == keyboard.Key.esc:
            return False
        try:
            k = key.char
        except:
            k = key.name
        if k == 'v':
            print('v_formation')
            self.v_formation()
        if k == 'r':
            print('rectangular_formation')
            self.rectangular_formation()
        if k == 'i':
            print('forward')
            self.follow_straight_line()
        if k == 'b':
            print('schedule')
            self.backend()
        if k == 'k':
            print('stop')
            self.stop_flag = 1
            self.stop_all()
        if k == 'h':
            print('home')
            self.go_home()
        if k == 't':
            print('head to head')
            self.head_to_head()


if __name__ == "__main__":
    nav = GoToPoints()
    listener = keyboard.Listener(on_press=nav.on_press)
    listener.start()
    if len(sys.argv) > 1:
        if sys.argv[1] == 'r':
            print('rectangular_formation')
            nav.rectangular_formation()
        if sys.argv[1] == 'v':
            print('v_formation')
            nav.v_formation()
        if sys.argv[1] == 'b':
            print('backend')
            nav.backend()
        if sys.argv[1] == 't':
            print('head to head')
            nav.head_to_head()
        if sys.argv[1] == 'h':
            print('home')
            nav.go_home()
        if sys.argv[1] == 'i':
            print('forward')
            nav.follow_straight_line()
    listener.join()
