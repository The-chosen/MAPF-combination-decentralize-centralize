# Import Robotarium Utilities
from pios_facility import PiosFacility
from utilities.transformations import *
from utilities.graph import *
from utilities.barrier_certificates import *
from utilities.misc import *
from utilities.controllers import *
import time
from follow_curve import FollowCurve


class Formation:
    def __init__(self):
        self.FollowCurve = FollowCurve()
        self.formation_control_gain = 1
        self.iterations = 700
        self.N = 5
        self.debug = 0
        # self.L = completeGL(5)
        self.L = np.array([
                            [2, -1, -1, 0, 0],
                            [-1, 2, 0, -1, 0],
                            [-1, 0, 2, 0, -1],
                            [0, -1, 0, 2, -1],
                            [0, 0, -1, -1, 2],
                        ])
        self.weights = np.array([])
        # Limit maximum linear speed of any robot
        self.magnitude_limit = 0.07
        if self.debug == 1:
            initial_conditions = np.array([[0, 0.2, 0, 0.2, 0.1], [0, 0, -0.2, -0.2, -0.1], [0, 0, 0, 0, 0]])
        else:
            initial_conditions = np.array([])
        self.goal = np.array([[1], [0.8], [0]])
        self.facility = PiosFacility(number_of_robots=self.N, show_figure=True, initial_conditions=initial_conditions,
                                     sim_in_real_time=True)
        self.facility.robot_id_tf(self.debug)
        self.si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()
        self.si_to_uni_dyn = create_si_to_uni_dynamics(linear_velocity_gain=0.5, angular_velocity_limit=0.8)

    def formation_control(self):

        if self.debug == 1:
            x = self.facility.get_poses()
            self.facility.step()
        else:
            x = self.facility.get_real_position()
        # Weight matrix to control inter-agent distances
        for i in range(self.N):
            for j in range(self.N):
                distance = np.linalg.norm(x[0:2, i] - x[0:2, j])
                self.weights = np.append(self.weights, distance)
        self.weights = self.weights.reshape((5, 5))
        for i in range(self.iterations):
            # Get the poses of the robots
            x = self.facility.get_poses()
            # Initialize a velocity vector
            dxi = np.zeros((2, self.N))
            for i in range(self.N):
                for j in topological_neighbors(self.L, i):
                    # Perform a weighted consensus to make the rectangular shape
                    error = x[:2, j] - x[:2, i]
                    dxi[:, i] += self.formation_control_gain * (
                            np.power(np.linalg.norm(error), 2) - np.power(self.weights[i, j], 2)) * error
            delta = self.goal[0:2, 0] - x[0:2, 0]
            norm_ = np.linalg.norm(delta)
            delta = [0.05, 0.03]
            # Threshold control inputs

            # Make sure that the robots don't collide
            # dxi = self.si_barrier_cert(dxi, x[:2, :])
            dxi[0, :] += delta[0]
            dxi[1, :] += delta[1]

            norms = np.linalg.norm(dxi, 2, 0)
            idxs_to_normalize = (norms > self.magnitude_limit)
            ratios = list(self.magnitude_limit / norms[idxs_to_normalize])
            ratios.append(1)
            ratio = min(ratios)
            dxi[:, :] *= ratio
            # Transform the single-integrator dynamcis to unicycle dynamics
            dxu = self.si_to_uni_dyn(dxi, x)
            print(dxu)
            # Set the velocities of the robots
            self.facility.set_velocities(np.arange(self.N), dxu)
            if self.debug == 1:
                self.facility.step()
            else:
                self.facility.step_real()
                time.sleep(0.033)
                self.facility.get_real_position()
        self.facility.stop_all()

    def formation_follow_curve(self, n):
        path = self.FollowCurve.get_path()
        new_path = self.FollowCurve.path_interpolation(path, n)
        temp1 = np.array([])
        temp2 = np.array([])
        temp3 = np.linspace(0, 0, num=self.N)
        for i in range(self.N):
            a = [new_path[0, -2 - i]]
            temp1 = np.append(temp1, [new_path[0, -2 - i]])
            temp2 = np.append(temp2, [new_path[1, -2 - i]])
        goal_points = np.array([temp1, temp2, temp3])

        if self.debug == 1:
            x = self.facility.get_poses()
            self.facility.step()
        else:
            x = self.facility.get_real_position()
        # Weight matrix to control inter-agent distances
        for i in range(self.N):
            for j in range(self.N):
                distance = np.linalg.norm(x[0:2, i] - x[0:2, j])
                self.weights = np.append(self.weights, distance)
        self.weights = self.weights.reshape((5, 5))

        for i in range(self.iterations):
            # Get the poses of the robots
            x = self.facility.get_poses()
            # Initialize a velocity vector
            dxi = np.zeros((2, self.N))
            current_point = np.delete(x, 2, 0)
            next_points, min_dis = self.FollowCurve.get_map_point(new_path, current_point, 0.1)
            if min_dis < n - 3:
                temp_point = np.array([[next_points[0]], [next_points[1]], [x[2, 0]]])
            else:
                temp_point = goal_points
                if np.linalg.norm(x[0:2, 0] - goal_points[0:2, 0]) < 0.04:
                    goal_points = x[0:2, 0]
            for i in range(self.N):
                for j in topological_neighbors(self.L, i):
                    # Perform a weighted consensus to make the rectangular shape
                    error = x[:2, j] - x[:2, i]
                    dxi[:, i] += self.formation_control_gain * (
                            np.power(np.linalg.norm(error), 2) - np.power(self.weights[i, j], 2)) * error
            distance_x = temp_point[0, 0] - x[0, 0]
            distance_y = temp_point[1, 0] - x[1, 0]
            # Threshold control inputs
            norms = np.linalg.norm(dxi, 2, 0)
            idxs_to_normalize = (norms > self.magnitude_limit)
            dxi[:, idxs_to_normalize] *= self.magnitude_limit / norms[idxs_to_normalize]
            # Make sure that the robots don't collide
            # dxi = self.si_barrier_cert(dxi, x[:2, :])
            dxi[0, :] += distance_x
            dxi[1, :] += distance_y
            # Transform the single-integrator dynamcis to unicycle dynamics
            dxu = self.si_to_uni_dyn(dxi, x)

            # Set the velocities of the robots
            self.facility.set_velocities(np.arange(self.N), dxu)
            if self.debug == 1:
                self.facility.step()
            else:
                self.facility.step_real()
                time.sleep(0.033)
                self.facility.get_real_position()
        self.facility.stop_all()


if __name__ == "__main__":
    fc = Formation()
    fc.formation_control()
