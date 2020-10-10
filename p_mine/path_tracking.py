#!/usr/bin/env python3
import numpy as np
from follow_curve import FollowCurve
from utilities.misc import *
from group_move import *
from pios_facility import PiosFacility
from get_init import pios_facility_parameter
import copy
import time
import threading

class Robot():
    def __init__(self,x = 0,y = 0,ang = 0,timestamp = 0):
        self.pos = np.array([[x],[y],[ang]])
        self.timestamp = timestamp
class Cooperate:
    def __init__(self):
        self.FollowCurve = FollowCurve()
        self.isStoped = True
        self.quit = False
        self.followcircle = False
        self.followcurve = False
        self.N = 4
        self.debug = 0
        self.linear_velocity_gain = 0.3
        self.angular_velocity_gain = 0.4
        self.velocity_magnitude_limit = 0.15
        self.angular_velocity_limit = 0.8
        self.position_error = 0.03
        self.position_epsilon = 0.01
        self.rotation_error = 0.125

        if self.debug == 1:
            initial_conditions = np.array([[0.3, 0.3, 0.1, 0.1], [0.1, 0.3, 0.3, 0.1], [0.0, 0, 0, 0]])
        else:
            initial_conditions = np.array([])
        self.goal = np.array([[2.0,1.7,2.0,1.7], [-0.1,-0.1,0.3,0.3], [0,0,0,0]])
        self.facility = PiosFacility(number_of_robots=self.N, show_figure=True, initial_conditions=initial_conditions,
                                     sim_in_real_time=True)
        self.facility.robot_id_tf(self.debug)
        self.pios_facility_parameter = pios_facility_parameter()
        self.controller = create_controller(self,"trigonometry")

    def get_circle(self,radius,pos):
        path = np.array([[], []])
        origin = np.array([pos[0]+radius,pos[1]])
        for i in range(31):
            theta = np.pi - np.pi/30 * i
            temp = np.array([[origin[0] + radius*np.cos(theta)], [origin[1]+radius*np.sin(theta)]])
            path = np.append(path, temp, axis=1)
        return path

    def generate_path_to_goal(self,current_point,goal_point):
        paths = []
        for i in range(current_point.shape[1]):
            path = np.array([[], []])
            dist = np.linalg.norm(current_point[0:2,i] - goal_point[0:2,i])
            segment = int(dist/0.02)
            temp_x = np.linspace(current_point[0,i],goal_point[0,i],segment)
            temp_y = np.linspace(current_point[1,i],goal_point[1,i],segment)
            path = np.array([temp_x,temp_y])
            paths.append(path)
        return paths

    def follow_circle(self,n,radius):
        if self.debug == 1:
            x = self.facility.get_poses()
            self.facility.step()
        else:
            x = self.facility.get_real_position()
        paths = []
        robots_goal_points = np.array([[],[],[]])
        local_points = copy.copy(x)
        for i in range(self.N):
            path = self.get_circle(radius,x[:,i])
            # new_path = self.FollowCurve.path_interpolation(path, n)
            goal_point = np.array([[path[0, -2]], [path[1, -2]], [0]])
            robots_goal_points = np.append(robots_goal_points,goal_point,axis=1)
            paths.append(path)
            delta = path[:,1]-path[:,0]
            angle = np.arctan2(delta[1],delta[0])
            local_points[2,i]=angle
        print(x)
        print("-------")
        print(local_points)
        while np.size(at_pose(x, local_points,self.position_error,self.rotation_error)) != self.N and self.isStoped == False:
            # Get poses of agents
            x = self.facility.get_poses()
            dxu = np.zeros((2,self.N))
            for i in range(self.N):

                ang = x[2,i]-local_points[2,i]
                ang = np.arctan2(np.sin(ang),np.cos(ang))
                dxu[0][i] = 0.0
                dxu[1][i] = -np.sign(ang) * 0.4
                if np.linalg.norm(x[2, i] - local_points[2, i]) < self.rotation_error:
                    local_points[:,i] = x[:,i]
                    dxu[1][i] = 0.0
            # print("Rotating dxu:", dxu)

            # Set the velocities by mapping the single-integrator inputs to unciycle inputs
            self.facility.set_velocities(np.arange(self.N), dxu)
            # Iterate the simulation
            if self.debug == 1:
                self.facility.step()
            else:
                self.facility.step_real()
                time.sleep(0.033)
                self.facility.get_real_position()
                # self.facility.poses_publisher()

        if self.quit or self.isStoped:
            return

        print("Rotation finished")

        while np.size(at_position(x[:2, :], robots_goal_points[:2,:],self.position_error)) != self.N and self.isStoped == False:
            x = self.facility.get_poses()
            current_points = np.delete(x, 2, 0)
            temp_points = np.array([[],[],[]])
            for i in range(self.N):

                current_point = np.reshape(current_points[:,i],(2,1))
                next_point, min_dis = self.FollowCurve.get_map_point(paths[i], current_point, 0.1)
                angle = np.arctan2(next_point[1]-x[1,i],next_point[0]-x[0,i])
                if min_dis < n - 3:
                    temp_point = np.array([[next_point[0]], [next_point[1]], [angle]])
                else:
                    temp_point = np.reshape(robots_goal_points[:,i],(3,1))
                    if np.linalg.norm(x[0:2,i] - robots_goal_points[0:2,i]) < self.position_error:
                        robots_goal_points[:,i] = x[:,i]
                temp_points=np.append(temp_points,temp_point,axis=1)

            # dxu = self.controller(x, robots_goal_points)
            dxu = self.controller(x, temp_points)
            # print("dxu: ",dxu)

            self.facility.set_velocities(np.arange(self.N), dxu)
            if self.debug == 1:
                self.facility.step()
            else:
                self.facility.step_real()
                time.sleep(0.033)
                self.facility.get_real_position()
        print("Translation finished")
        self.facility.stop_all()
        self.followcircle = False

    def formation_follow_curve(self, n):
        for i in range(self.goal.shape[1]):
            if self.debug == 1:
                x = self.facility.get_poses()
                print(x)
                self.facility.step()
            else:
                x = self.facility.get_real_position()

            dist, angle_to_destination = get_dist_and_angle(x,self.goal[:,i])
            robots_goal_x = x[0,:]+ dist*np.cos(angle_to_destination)
            robots_goal_y = x[1,:]+ dist*np.sin(angle_to_destination)
            robots_goal_z = np.linspace(angle_to_destination,angle_to_destination,self.N)
            robots_goal_points = np.array([robots_goal_x,robots_goal_y,robots_goal_z])
            
            print("angle:", angle_to_destination*180/np.pi)
            local_points = copy.copy(x)
            local_points[2,:] = angle_to_destination

            while np.size(at_pose(x, local_points,self.position_error,self.rotation_error)) != self.N and self.isStoped == False:
                # Get poses of agents
                x = self.facility.get_poses()
                dxu = np.zeros((2,self.N))
                for i in range(self.N):

                    ang = x[2,i]-local_points[2,i]
                    ang = np.arctan2(np.sin(ang),np.cos(ang))
                    dxu[0][i] = 0.0
                    dxu[1][i] = -np.sign(ang) * 0.4
                    if np.linalg.norm(x[2, i] - local_points[2, i]) < self.rotation_error:
                        local_points[:,i] = x[:,i]
                        dxu[1][i] = 0.0
                # print("Rotating dxu:", dxu)

                # Set the velocities by mapping the single-integrator inputs to unciycle inputs
                self.facility.set_velocities(np.arange(self.N), dxu)
                # Iterate the simulation
                if self.debug == 1:
                    self.facility.step()
                else:
                    self.facility.step_real()
                    time.sleep(0.033)
                    self.facility.get_real_position()
                    # self.facility.poses_publisher()

            if self.quit or self.isStoped:
                return
            print("Rotation finished")

            paths = self.generate_path_to_goal(x,robots_goal_points)        
            new_paths = []
            goal_points = []

            for i in range(self.N):
                new_path = self.FollowCurve.path_interpolation(paths[i], n)
                goal_point = np.array([new_path[0, -2], new_path[1, -2], angle_to_destination])
                new_paths.append(new_path)
                goal_points.append(goal_point)

            if self.debug == 1:
                x = self.facility.get_poses()
                self.facility.step()
            else:
                x = self.facility.get_real_position()
            
            while np.size(at_position(x[:2, :], robots_goal_points[:2,:],self.position_error)) != self.N and self.isStoped == False:
                # Get the poses of the robots
                x = self.facility.get_poses()
                current_points = np.delete(x, 2, 0)
                temp_points = np.array([[],[],[]])
                for i in range(self.N):

                    current_point = np.reshape(current_points[:,i],(2,1))
                    next_point, min_dis = self.FollowCurve.get_map_point(new_paths[i], current_point, 0.1)
                    angle = np.arctan2(next_point[1]-x[1,i],next_point[0]-x[0,i])
                    if min_dis < n - 3:
                        temp_point = np.array([[next_point[0]], [next_point[1]], [angle]])
                    else:
                        temp_point = np.reshape(robots_goal_points[:,i],(3,1))
                        if np.linalg.norm(x[0:2,i] - robots_goal_points[0:2,i]) < self.position_error:
                            robots_goal_points[:,i] = x[:,i]
                    temp_points=np.append(temp_points,temp_point,axis=1)

                # dxu = self.controller(x, robots_goal_points)
                dxu = self.controller(x, temp_points)
                # print("dxu: ",dxu)

                self.facility.set_velocities(np.arange(self.N), dxu)
                if self.debug == 1:
                    self.facility.step()
                else:
                    self.facility.step_real()
                    time.sleep(0.033)
                    self.facility.get_real_position()
            print("Translation finished")
        print("Movement finished")
        self.facility.stop_all()
        self.followcurve = False

    def monitor(self):
        while not self.quit:
            if self.followcurve:
                t3 = threading.Thread(target=self.formation_follow_curve(50), name='Follow curve')
                t3.start()
                t3.join()
            elif self.followcircle:
                t4 = threading.Thread(target=self.follow_circle(50,0.25), name='Follow circle')
                t4.start()
                t4.join()


    def Keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

    def on_press(self,key):
        if key==keyboard.Key.esc:
            return False
        try:
            k = key.char
        except:
            k = key.name
        if k == 'q':
            print('quit')
            self.quit = True
            self.isStoped = True
            self.followcircle = False
            self.followcurve = False
            for i in range(2):
                self.facility.stop_all()
            return False

        if k == 'i':
            print('forward')
            self.followcurve = False
            self.followcircle = False
            self.isStoped = True
            for i in range(2):
                self.facility.get_poses()
                self.facility.get_real_position()
                dxu = np.array([[0.015,0.015,0.015,0.015],[0.0,0.0,0.0,0.0]])
                self.facility.set_velocities(np.arange(self.N), dxu)
                self.facility.step_real()
                time.sleep(0.033)
        if k == 'r':
            print('rotation')
            self.isStoped = True
            self.followcurve = False
            self.followcircle = False
            for i in range(2):
                self.facility.get_poses()
                self.facility.get_real_position()
                dxu = np.array([[0.0,0.0,0.0,0.0],[0.3,0.3,0.3,0.3]])
                self.facility.set_velocities(np.arange(self.N), dxu)
                self.facility.step_real()
                time.sleep(0.033)
        if k == 'k':
            print('stop')
            self.isStoped = True
            self.followcurve = False
            self.followcircle = False
            for i in range(2):
                self.facility.stop_all()
                time.sleep(0.033)
        if k == 'f':
            print("follow curve")
            self.isStoped = False
            self.followcurve = True
        
        if k == 'c':
            print("follow circle")
            self.isStoped = False
            self.followcircle = True
            # self.follow_circle(50,0.25)
        
if __name__ == "__main__":
    nav = Cooperate()
    t1 = threading.Thread(target=nav.monitor, name='monitor')
    t2 = threading.Thread(target=nav.Keyboard_listener, name='keyboard')
    t1.start()
    t2.start()
    t2.join()
    t1.join()
