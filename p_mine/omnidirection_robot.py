#!/usr/bin/env python3
import numpy as np
from follow_curve import FollowCurve
from utilities.misc import *
from omnidirection_facility import *
import time
import copy
import signal

def signal_handler(signal, frame):
           global interrupted
           interrupted = True

signal.signal(signal.SIGINT, signal_handler)
interrupted = False
class OmniRobot:
    def __init__(self):
        self.length = 0.70
        self.width = 0.50

        self.use_QR = 0

        self.linear_velocity_gain = 0.5
        self.position_error = 0.03
        self.angular_error = 0.4

        self.facility = OmnidirectionFacility()

        self.roller_map, self.unit_map = self.facility.get_map()

        self.FollowCurve = FollowCurve()

    def get_roller_id_QR(self,pose):
        x_min = -self.width/2
        x_max =  self.width/2
        y_min = -self.length/2
        y_max =  self.length/2
        indexes = []
        x = np.zeros((3,1))
        for i in range(pose.shape[1]):
            index = np.array([])
            index = index.astype('int32')
            for roller in self.roller_map:
                x_new = -(roller.x - pose[0][i])*np.cos(pose[2][i])+(roller.y - pose[1][i])*np.sin(pose[2][i])
                y_new = -(roller.x - pose[0][i])*np.sin(pose[2][i])-(roller.y - pose[1][i])*np.cos(pose[2][i])
                if x_min <= x_new <= x_max and y_min <= y_new <= y_max:
                    index = np.append(index,np.array([roller.id-1]))
            if index.size != 0:
                indexes.append(index)
                x[0] = pose[0][i]
                x[1] = pose[1][i]
                x[2] = pose[2][i]
        return indexes, x

    def get_path_to_goal(self,x,goal,n):

        path = np.array([[], []])
        goal_point = np.delete(goal, 2, 0)
        dist = np.linalg.norm(x[0:2] - goal_point)
        segment = int(dist/0.02)
        temp_x = np.transpose(np.linspace(x[0],goal[0],segment))
        temp_y = np.transpose(np.linspace(x[1],goal[1],segment))
        path = np.append(temp_x,temp_y,axis = 0)
        new_path = self.FollowCurve.path_interpolation(path, n)
        wrapped = np.arctan2(np.sin(goal[2]),np.cos(goal[2]))
        goal_point = np.array([np.array([new_path[0, -2]]), np.array([new_path[1, -2]]), wrapped])
        goal_point = np.reshape(goal_point,(3,1)) 
        return new_path, goal_point

    def box_to_roller(self,index,x,dxi):
        for i in range(len(index)):
            roller = self.roller_map[index[i]]
            R = np.sqrt(pow(x[0]-roller.x,2),pow(x[1]-roller.y,2)) # Distance of roller to the center of box
            ang_vel = R * dxi[2] # Angular velocity at roller due to box rotation
            rot_world_angle = np.arctan2(x[1] - roller.y,x[0] - roller.x) + np.pi/2 # Roller angle in world coordinate
            rot_world_angle = np.arctan2(np.sin(rot_world_angle),np.cos(rot_world_angle))
            rotation_vel = np.cos(roller.angle-rot_world_angle)*ang_vel # x velocity in roller coordinate
            # rotation_vel = np.cos(roller.angle+np.pi/2-rot_world_angle)*ang_vel # x velocity in roller coordinate
            trans_vel = np.sin(x[2]+roller.angle) * dxi[0] - np.cos(x[2]+roller.angle) * dxi[1]
            roller.velocity = trans_vel + rotation_vel

    def move(self,goal_unit, n):
        count = 0
        while not interrupted:
            if self.use_QR == 1:
                x = self.facility.get_real_position()
                # x = self.facility.get_poses()
                indexes, x = self.get_roller_id_QR(x)
            else:
                x = self.facility.get_real_position_from_contour()
                indexes = self.facility.get_roller_id()
                x = np.reshape(x[:,0],(3,1))
            
            #reset roller velocity
            for i in range(len(self.roller_map)):
                self.roller_map[i].velocity = 0
            
            self.facility.set_velocities(self.roller_map)
            # self.facility.step_real()
            self.facility.step_real2()
            # self.facility.recviceMsg()
            # print("set vel")
            # return
            if len(indexes) != 0:
                start_angle = x[2][0]

                # robot_id = self.facility.get_robot_id(self.debug)

                # if int(robot_id[-1])%2 == 0:
                # if count%2 == 0:
                #     goal_unit = 4
                #     offset_x = 0.05
                #     offset_y = 0.0
                #     print("Goal 1")
                # else:
                #     goal_unit = 7
                #     offset_x = -0.05
                #     offset_y = 0.0
                #     print("Goal 2")
                # count +=1 
                if count % 2 ==0:
                    goal_unit = 8
                    offset_x = 0.1
                    offset_y = 0.05
                else:
                    goal_unit = 3
                    offset_x = -0.1
                    offset_y = -0.05
                count +=1 
                print("goal_unit: ",goal_unit)
                goal_position = copy.copy(self.unit_map[goal_unit])
                goal_position[0] += offset_x
                goal_position[1] += offset_y
                goal = np.reshape(np.append(goal_position,start_angle),(3,1))

                dxi = np.zeros((3,x.shape[1]))
                path, goal_point = self.get_path_to_goal(x,goal,n)
                new_path = path
                # new_goal = copy.copy(goal)
                # new_goal[1] += 0.8
                # new_path, goal_point = self.get_path_to_goal(goal,new_goal,n)
                # new_path = np.append(path,new_path,axis=1)
                cur_error,error_old,error_older = 0,0,0
                Kp,Ki,Kd = 2.0,0.02,0.05
                while np.size(at_position(x[:2], goal_point[:2],self.position_error)) != x.shape[1] and len(indexes) != 0 and not interrupted:

                    if self.use_QR == 1:
                        x = self.facility.get_real_position()
                        indexes,x = self.get_roller_id_QR(x)
                    else:
                        x = self.facility.get_real_position_from_contour()
                        indexes = self.facility.get_roller_id()
                        x = np.reshape(x[:,0],(3,1))

                    if len(indexes) == 0:
                        break
                    print("indexes: \n",indexes)
                    # continue
                    print("x:n \n",x)
                    print("goal\n",goal_point)
                    #reset roller velocity
                    for i in range(len(self.roller_map)):
                        self.roller_map[i].velocity = 0
                                        
                    current_points = np.delete(x, 2, 0)
                    
                    current_point = np.reshape(current_points[:],(2,1))
                    
                    next_point, min_dis = self.FollowCurve.get_map_point(new_path, current_point, 0.2)
                    angle = np.arctan2(next_point[1]-x[1],next_point[0]-x[0])
                    if min_dis < n - 5:
                        temp_point = np.array([[next_point[0]], [next_point[1]], angle])
                    else:
                        temp_point = np.reshape(goal_point,(3,1))
                        if np.linalg.norm(x[0:2] - goal_point[0:2,0]) < self.position_error:
                            goal_point = x[:]
                    
                    ang_now = x[2]
                    ang_diff =  start_angle - ang_now 
                    cur_error = np.arctan2(np.sin(ang_diff),np.cos(ang_diff))
                    delta_ang = Kp*(error_old-cur_error)+Ki*cur_error+Kd*(cur_error-2*error_old+error_older)
                    
                    error_old = cur_error
                    error_older = error_old
                    dxi[2] += delta_ang

                    dx = self.linear_velocity_gain * (temp_point[0]-x[0])#*cos(ang_diff)
                    dy = self.linear_velocity_gain * (temp_point[1]-x[1])#*cos(ang_diff)

                    dxi[0] = np.cos(x[2])*dx - np.sin(x[2])*dy
                    dxi[1] = np.sin(x[2])*dx + np.cos(x[2])*dy
                    print("dxi:\n",dxi)
                    self.box_to_roller(indexes[0],x,dxi)

                    if self.use_QR == 1:
                        self.facility.set_velocities(self.roller_map)
                        self.facility.step_real()
                        time.sleep(0.033)
                        x = self.facility.get_real_position()
                        indexes,x = self.get_roller_id_QR(x)
                    else:
                        self.facility.set_velocities(self.roller_map)
                        self.facility.step_real2()
                        time.sleep(0.033)
                        x = self.facility.get_real_position_from_contour()
                        indexes = self.facility.get_roller_id()
                        x = np.reshape(x[:,0],(3,1))
                print("Out")

            if interrupted:
                for i in range(len(self.roller_map)):
                    self.roller_map[i].velocity = 0
                self.facility.set_velocities(self.roller_map)
                # self.facility.step_real()
                x = self.facility.get_real_position_from_contour()
                self.facility.step_real2()
                print("Stopped by keyboard")


if __name__ == "__main__":
    nav = OmniRobot()
    goal_unit = 2
    nav.move(goal_unit, 30)
