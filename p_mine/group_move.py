#!/usr/bin/env python3

from go_to_points import *

from utilities.controllers import *
class GroupMove:
    def init_pid(self,nav):
        self.error_old = np.zeros((2,self.N))
        self.error_older = np.zeros((2,self.N))
        self.dxu_old = np.zeros((2,self.N))
        self.cur_error = np.zeros((2,self.N))
        self.vel_gain = nav.pios_facility_parameter.linear_velocity_gain
        self.ang_gain = nav.pios_facility_parameter.angular_velocity_gain
        self.angular_velocity_limit = nav.pios_facility_parameter.angular_velocity_limit
        self.velocity_magnitude_limit = nav.pios_facility_parameter.velocity_magnitude_limit



    def group_move(self,goal_points):
        nav = GoToPoints()
        self.N = nav.N
        assert self.N is 4, "The number of robots must be 4. Recieved %r." % self.N
        self.controller = create_controller(nav.pios_facility_parameter,"tracking") # trigonometry or tracking
        init_poses = self.generate_init_positions()
        print("=== Moving to start position ===")
        nav.move_to(init_poses)
        nav.stop_all

        print("=== Moving to Goal position ===")
        
        if nav.debug == 1:
            x = nav.facility.get_poses()
            nav.facility.step()
        else:
            x = nav.facility.get_real_position()

        for i in range(goal_points.shape[1]):
            goal_point = goal_points[:,i]
            print("Current goal_point is: ", goal_point)
            dist, angle_to_destination = get_dist_and_angle(x,goal_point)
        
            local_points = np.zeros((3,self.N))
            print("angle_to_destination",angle_to_destination)
            for i in range(self.N):
                local_points[0,:] = x[0,:]
                local_points[1,:] = x[1,:]
                local_points[2,i] = angle_to_destination

            dxu = np.zeros((2, self.N))
            while np.size(at_pose(x, local_points,nav.position_error,nav.rotation_error)) != self.N and nav.stop_flag == 0:
                # Get poses of agents
                x = nav.facility.get_poses()
                for i in range(self.N):
                    dxu[0][i] = 0.0
                    dxu[1][i] = 0.4
                    if np.linalg.norm(x[2, i] - local_points[2, i]) < nav.rotation_error:
                        local_points[2][i] = x[2][i]
                        dxu[1][i] = 0.0
                # dxu = nav.unicycle_pose_controller(x, local_points)

                # Set the velocities by mapping the single-integrator inputs to unciycle inputs
                nav.facility.set_velocities(np.arange(self.N), dxu)
                # Iterate the simulation
                if nav.debug == 1:
                    nav.facility.step()
                else:
                    nav.facility.step_real()
                    time.sleep(0.033)
                    nav.facility.get_real_position()
                    nav.facility.poses_publisher()

            print("Rotation finished")
            
            self.init_pid(nav)
            robots_goal_points = np.zeros((2,self.N))
            # for i in range(goal_points.shape[1]):
            robots_goal_x = x[0,:]+ dist*np.cos(angle_to_destination)
            robots_goal_y = x[1,:]+ dist*np.sin(angle_to_destination)
            robots_goal_z = np.arctan2((robots_goal_y-x[1,:]),(robots_goal_x-x[0,:]))
            robots_goal_points = np.array([robots_goal_x,robots_goal_y,robots_goal_z])
            print("robots goal points: ",robots_goal_points)
            paths = generate_path_to_goal(x,robots_goal_points)
            min_dist = np.linalg.norm(robots_goal_points[:2, :] - x[:2, :], 2, 0)
            while np.size(at_position(x[:2, :], robots_goal_points[:2,:],0.03)) != self.N and nav.stop_flag == 0:
                # Get poses of agents
                x = nav.facility.get_poses()
                # print("Distance: ", dist)
                # print("Each robot position error: ",np.linalg.norm(x[0:2]-robots_goal_points[0:2],axis = 0))
                # trigonometry_control
                dxu = self.controller(x, robots_goal_points)
                cur_dist = np.linalg.norm(robots_goal_points[:2, :] - x[:2, :], 2, 0)
                index1 = cur_dist < min_dist
                min_dist[index1] = cur_dist[index1]
                index = cur_dist > min_dist
                
                if np.size(np.nonzero(index)) != 0:
                    robots_goal_points[:,(index)] = x[:,(index)]
                    dxu[:,index] = np.zeros((2, 1))
                # pid control
                # dxu = self.pid_controller(x, robots_goal_points,angle_to_destination)                
                # self.update_pid(dxu)
                
                # print("dxu : ", dxu[:,0])
                # Set the velocities by mapping the single-integrator inputs to unciycle inputs
                nav.facility.set_velocities(np.arange(self.N), dxu)
                # Iterate the simulation
                if nav.debug == 1:
                    nav.facility.step()
                else:
                    nav.facility.step_real()
                    time.sleep(0.033)
                    nav.facility.get_real_position()
                    nav.facility.poses_publisher()
            print("Translation finished")
            x_c = x.min(1)[0]+0.5*(x.max(1)[0]-x.min(1)[0])
            y_c = x.min(1)[1]+0.5*(x.max(1)[1]-x.min(1)[1])
            print("current position: ", [x_c, y_c])
            print("goal position: ", goal_point)
        nav.stop_all()
    
    def pid_controller(self, x, robots_goal_points, ang_require):
        Kp_vel = 0.8
        Ki_vel = 0.01
        Kd_vel = 0.1
        Kp = 0.8
        Ki = 0.001
        Kd = 0.1
        dxu = np.zeros((2,self.N))
        for i in range(self.N):
            distance = np.linalg.norm(robots_goal_points[:2,i]-x[:2,i])
            self.cur_error[0,i] = self.vel_gain*distance - self.dxu_old[0,i]
            delta_vel = Kp_vel*(self.cur_error[0,i]-self.error_old[0,i])+Ki_vel*self.cur_error[0,i]+Kd_vel*(self.cur_error[0,i]-2*self.error_old[0,i]+self.error_older[0,i])
            dxu[0,i] = self.dxu_old[0,i]+delta_vel
            ang_now = x[2,i]
            ang_diff = ang_require - ang_now
            self.cur_error[1,i] = np.arctan2(np.sin(ang_diff),np.cos(ang_diff))
            delta_ang = Kp*(self.cur_error[1,i]-self.error_old[1,i])+Ki*self.cur_error[1,i]+Kd*(self.cur_error[1,i]-2*self.error_old[1,i]+self.error_older[1,i])
            # print(delta_dxu)
            dxu[1,i] = self.dxu_old[1,i] + delta_ang
        print("delta_Kp: ", Kp*(self.cur_error[1,0]-self.error_old[1,0]))
        print("delta_Ki: ", Ki*self.cur_error[1,0])
        print("delta_Kd: ", Kd*(self.cur_error[1,i]-2*self.error_old[1,i]+self.error_older[1,i]))
        print("self.dxu_old: ", self.dxu_old[1,0])
        print("delta_ang: ", (dxu[1,0]-self.dxu_old[1,0]))
        dxu[0,dxu[0,:]>self.velocity_magnitude_limit] = self.vel_gain*self.velocity_magnitude_limit
        dxu[0,dxu[0,:]<-self.velocity_magnitude_limit] = -self.vel_gain*self.velocity_magnitude_limit
        dxu[1,dxu[1,:]>self.angular_velocity_limit] = self.ang_gain*self.angular_velocity_limit
        dxu[1,dxu[1,:]<-self.angular_velocity_limit] = -self.ang_gain*self.angular_velocity_limit
        return dxu
    
    def update_pid(self,dxu):
        self.dxu_old = dxu
        self.error_old = self.cur_error
        self.error_older = self.error_old

    def generate_init_positions(self):
        # Generate initial position for simulation
        x = [ 0.3, 0.3, 0.1, 0.1]
        y = [ 0.1, 0.3, 0.3, 0.1]
        z = np.linspace(0,0,self.N)
        poses = np.array([x,y,z])
        return poses
    
def create_controller(nav,controller_name):
    if controller_name == "trigonometry":
        controller = create_trigonometry_controller(
            linear_vel_gain=nav.linear_velocity_gain,
            angular_vel_gain=nav.angular_velocity_gain,
            velocity_magnitude_limit=nav.velocity_magnitude_limit,
            angular_velocity_limit=nav.angular_velocity_limit,
            position_error=nav.position_error,
            rotation_error=nav.rotation_error)
    elif controller_name == "tracking":
        controller = create_tracking_controller(
            linear_vel_gain=nav.linear_velocity_gain,
            angular_vel_gain=nav.angular_velocity_gain,
            velocity_magnitude_limit=nav.velocity_magnitude_limit,
            angular_velocity_limit=nav.angular_velocity_limit,
            position_error=nav.position_error,
            rotation_error=nav.rotation_error)
    return controller
    
def get_dist_and_angle(poses,goal_point):
        x_c = poses.min(1)[0]+0.5*(poses.max(1)[0]-poses.min(1)[0])
        y_c = poses.min(1)[1]+0.5*(poses.max(1)[1]-poses.min(1)[1])
        position_error = goal_point[:2]-np.array([x_c,y_c])
        dist = np.linalg.norm(position_error)
        angle = np.arctan2((goal_point[1]-y_c),(goal_point[0]-x_c))
        return dist, angle     



def generate_path_to_goal(current_point,goal_point):
    paths = []
    for i in range(current_point.shape[1]):
        path = np.array([[], []])
        dist = np.linalg.norm(current_point[0:2,i] - goal_point[0:2,i])
        segment = int(dist/0.2)
        temp_x = np.linspace(current_point[0,i],goal_point[0,i],segment)
        temp_y = np.linspace(current_point[1,i],goal_point[1,i],segment)
        path = np.array([temp_x,temp_y])
        paths.append(paths)
    return paths

if __name__ == "__main__":
    gm = GroupMove()
    goal_position_x = [0.3, 0.8, 0.8, 0.3, 0.3]
    goal_position_y = [0.3, 0.3, 0.7, 0.7, 0.3]
    goal_pose = [0, 0, 0, 0, 0]
    goal_points = np.array([goal_position_x,goal_position_y,goal_pose])

    gm.group_move(goal_points)
