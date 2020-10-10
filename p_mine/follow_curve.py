from path_interpolation import PathInterpolation
from go_to_points import *
import math


class FollowCurve:
    def get_path(self):
        path = np.array([[], []])
        for i in range(int(3.1416 / 0.1)):
            theta = 3.1416 - 0.1 * i
            temp = np.array([[0.6 + math.cos(theta)], [math.sin(theta)]])
            path = np.append(path, temp, axis=1)
        return path

    def path_interpolation(self, path, n):
        path_interp = PathInterpolation(path)
        new_path = path_interp.path_normalize(n)
        return new_path

    def get_map_point(self, new_path, current_points, delta):
        temp = np.reshape(current_points[:, 0], (2, 1))
        subtraction = new_path[:, 0:-1] - temp
        dis = np.sqrt(np.sum(subtraction * subtraction, axis=0))
        min_dis = np.argmin(dis)
        if min_dis == 0:
            min_dis = 1
        if min_dis == len(new_path[1, :]) - 2:
            min_dis = len(new_path[1, :]) - 3
        line_point1 = new_path[:, min_dis + 1]
        line_point2 = new_path[:, min_dis]
        u = line_point1
        v = line_point2
        x = np.array([current_points[0][0], current_points[1][0]])
        n = u - v
        n /= np.linalg.norm(n, 2)
        goal_points = u + n * np.dot(x - u, n) + n * delta
        return goal_points, min_dis

    def follow_curve(self, n):
        nav = GoToPoints()
        debug = nav.model()
        path = self.get_path()
        new_path = self.path_interpolation(path, n)
        temp1 = np.array([])
        temp2 = np.array([])
        temp3 = np.linspace(0, 0, num=nav.N)
        for i in range(nav.N):
            a = [new_path[0, -2 - i]]
            temp1 = np.append(temp1, [new_path[0, -2-i]])
            temp2 = np.append(temp2, [new_path[1, -2-i]])
        goal_points = np.array([temp1, temp2, temp3])
        if debug == 1:
            x = nav.facility.get_poses()
            nav.facility.step()
        else:
            x = nav.facility.get_real_position()
        while np.size(at_pose(x, goal_points)) != nav.N:
            current_point = np.delete(x, 2, 0)
            next_points, min_dis = self.get_map_point(new_path, current_point, 0.1)
            temp_point = np.array([[next_points[0]], [next_points[1]], [x[2, 0]]])
            if min_dis < n-3:
                nav.follow(temp_point)
            else:
                nav.follow(goal_points)
                if np.linalg.norm(x[0:2, :] - goal_points[0:2, :]) < 0.04:
                    goal_points = x
            if debug == 1:
                x = nav.facility.get_poses()
                nav.facility.step()
            else:
                x = nav.facility.get_real_position()
        nav.stop_all()


if __name__ == "__main__":
    fc = FollowCurve()
    fc.follow_curve(50)
