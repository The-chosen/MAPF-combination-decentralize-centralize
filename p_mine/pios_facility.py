#!/usr/bin/env python3
import logging
logging.basicConfig(level = logging.INFO,format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)
import math
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from robotarium_abc import *
import json
from cv_bridge import CvBridge, CvBridgeError
import cv2


class PiosFacility(RobotariumABC):
    def __init__(self, number_of_robots=-1, show_figure=True, sim_in_real_time=True, initial_conditions=np.array([])):
        super().__init__(number_of_robots, show_figure, sim_in_real_time, initial_conditions)

        with open('./robot_id_ip_mapping.json', 'r')as fp:
            self.robot_ip = json.load(fp)
            print(self.robot_ip)

        # Initialize some rendering variables
        self.previous_render_time = time.time()
        self.sim_in_real_time = sim_in_real_time

        # Initialize checks for step and get poses calls
        self._called_step_already = True
        self._checked_poses_already = False

        # Initialization of error collection.
        self._errors = {}

        # Initialize steps
        self._iterations = 0
        self._iterations = 0
        self.number_of_robots = number_of_robots

        # PiOSFacility This object provides routines to interface with the PiOSFacility.
        self.bridge = CvBridge()
        self.points_list = []
        self.ros_sub = {}
        self.contours = {}
        self.obstacle = {}
        self.tf_id = {}
        self.get_goals = np.zeros((3, self.number_of_robots))
        rospy.init_node('pios_topic_robot_node', anonymous=True)
        rospy.Subscriber('pios_topic_robot_pos', String, self.ros_callback)
        self.ros_msg = rospy.Publisher('pios_topic_command', String, queue_size=100)
        rospy.Subscriber('pios_topic_detect_obstacle', Image, self.image_callback)
        rospy.Subscriber('pios_topic_move', String, self.poses_callback)
        self.ros_poses = rospy.Publisher('pios_topic_move_pos', String, queue_size=10)

    def poses_publisher(self):
        msg = self.ros_sub
        while len(msg) == 0:
            msg = self.ros_sub
        tmp = msg
        ids = list(tmp.keys())
        for i in range(len(ids)):
            robot_id = ids[i]
            for j in range(self.number_of_robots):
                if self.tf_id[j] == robot_id:
                    data = {
                        'action': 'get_position',
                        'robot_id': robot_id,
                        'x': self.poses[0, i],
                        'y': self.poses[1, i],
                        'angular': self.poses[2, i]
                    }
                    data = json.dumps(data)
                    self.ros_poses.publish(data)

    def poses_callback(self, poses):
        if poses:
            tmp = json.loads(poses.data)
            robot_id = 'robot_'+tmp['robot_id']
            destination = tmp['destination']
            x = destination[0]
            y = destination[1]
            # angle = destination[2]
            for j in range(self.number_of_robots):
                if self.tf_id[j] == robot_id:
                    self.get_goals[0, j] = x
                    self.get_goals[1, j] = y
                    self.get_goals[2, j] = 0

    def get_goals_position(self):
        return self.get_goals

    def get_debug_position(self):
        return self.poses

    def ros_callback(self, msg):
        # print(msg.data)
        self.ros_sub = json.loads(msg.data)

    def image_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            pass
        self.points_list = []
        imageHeight, imageWidth = cv_img.shape[0:2]
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([156, 43, 46])
        high_hsv = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_hsv, high_hsv)
        lower_hsv1 = np.array([0, 43, 46])
        high_hsv1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_hsv1, high_hsv1)
        mask2 = mask1 + mask
        img_mean = cv2.medianBlur(mask2, 5)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        dilation = cv2.dilate(img_mean, kernel)
        contours, h = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            a = contours[i][0][0][0]
            b = contours[i][0][0][1]
            if a < 228 or a > 1471:
                continue
            if b < 87 or b > 750:
                continue
            area = cv2.contourArea(contours[i])
            imageArea = imageWidth * imageHeight
            point = []
            if area > imageArea * 0.0007:
                rect = cv2.minAreaRect(contours[i])
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                # cv2.circle(cv_img, (box[0][0], box[0][1]), 3, (0, 0, 255), 2)
                # cv2.circle(cv_img, (box[1][0], box[1][1]), 3, (0, 0, 255), 2)
                # cv2.circle(cv_img, (box[2][0], box[2][1]), 3, (0, 0, 255), 2)
                # cv2.circle(cv_img, (box[3][0], box[3][1]), 3, (0, 0, 255), 2)
                for j in range(len(box)):
                    tmp = self.pointsToWorld(box[j])
                    point.append(tmp[0])
                    point.append(tmp[1])
                self.points_list.append(point)
        # cv2.namedWindow('input_image', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("input_image", cv_img)
        # cv2.waitKey(3)

    def get_poses(self):
        """Returns the states of the agents.

        -> 3xN numpy array (of robot poses)
        """

        assert (not self._checked_poses_already), "Can only call get_poses() once per call of step()."
        # Allow step() to be called again.
        self._called_step_already = False
        self._checked_poses_already = True

        return self.poses

    def call_at_scripts_end(self):
        """Call this function at the end of scripts to display potentail errors.
        Even if you don't want to print the errors, calling this function at the
        end of your script will enable execution on the Robotarium testbed.
        """
        print('##### DEBUG OUTPUT #####')
        print('Your simulation will take approximately {0} real seconds when deployed on the Robotarium. \n'.format(
            math.ceil(self._iterations * 0.033)))

        if bool(self._errors):
            if "boundary" in self._errors:
                print('\t Simulation had {0} {1}\n'.format(self._errors["boundary"], self._errors["boundary_string"]))
            if "collision" in self._errors:
                print('\t Simulation had {0} {1}\n'.format(self._errors["collision"], self._errors["collision_string"]))
            if "actuator" in self._errors:
                print('\t Simulation had {0} {1}'.format(self._errors["actuator"], self._errors["actuator_string"]))
        else:
            print('No errors in your simulation! Acceptance of your experiment is likely!')

        return

    def step(self):
        """Increments the simulation by updating the dynamics.
        """
        assert (not self._called_step_already), "Make sure to call get_poses before calling step() again."

        # Allow get_poses function to be called again.
        self._called_step_already = True
        self._checked_poses_already = False

        # Validate before thresholding velocities
        self._errors = self._validate()
        self._iterations += 1
        # Update dynamics of agents
        self.poses[0, :] = self.poses[0, :] + self.time_step * np.cos(self.poses[2, :]) * self.velocities[0, :]
        self.poses[1, :] = self.poses[1, :] + self.time_step * np.sin(self.poses[2, :]) * self.velocities[0, :]
        self.poses[2, :] = self.poses[2, :] + self.time_step * self.velocities[1, :]
        # Ensure angles are wrapped
        self.poses[2, :] = np.arctan2(np.sin(self.poses[2, :]), np.cos(self.poses[2, :]))
        # Update graphics

        if self.show_figure:
            if self.sim_in_real_time:
                t = time.time()
                while t - self.previous_render_time < self.time_step:
                    t = time.time()
                self.previous_render_time = t
            for i in range(self.number_of_robots):
                self.chassis_patches[i].center = self.poses[:2, i]
                self.chassis_patches[i].orientation = self.poses[2, i] + math.pi / 4

                self.right_wheel_patches[i].center = self.poses[:2, i] + self.robot_radius * np.array(
                    (np.cos(self.poses[2, i] + math.pi / 2), np.sin(self.poses[2, i] + math.pi / 2))) + \
                                                     0.04 * np.array(
                    (-np.sin(self.poses[2, i] + math.pi / 2), np.cos(self.poses[2, i] + math.pi / 2)))
                self.right_wheel_patches[i].orientation = self.poses[2, i] + math.pi / 4

                self.left_wheel_patches[i].center = self.poses[:2, i] + self.robot_radius * np.array(
                    (np.cos(self.poses[2, i] - math.pi / 2), np.sin(self.poses[2, i] - math.pi / 2))) + \
                                                    0.04 * np.array(
                    (-np.sin(self.poses[2, i] + math.pi / 2), np.cos(self.poses[2, i] + math.pi / 2)))
                self.left_wheel_patches[i].orientation = self.poses[2, i] + math.pi / 4

                self.right_led_patches[i].center = self.poses[:2, i] + 0.75 * self.robot_radius * np.array(
                    (np.cos(self.poses[2, i]), np.sin(self.poses[2, i]))) - \
                                                   0.04 * np.array(
                    (-np.sin(self.poses[2, i]), np.cos(self.poses[2, i])))
                self.left_led_patches[i].center = self.poses[:2, i] + 0.75 * self.robot_radius * np.array(
                    (np.cos(self.poses[2, i]), np.sin(self.poses[2, i]))) - \
                                                  0.015 * np.array(
                    (-np.sin(self.poses[2, i]), np.cos(self.poses[2, i])))

            self.figure.canvas.draw_idle()
            self.figure.canvas.flush_events()

    def pointsToWorld(self, points):
        # T = np.array([
        #     [1.0012, -0.0009, -0.0017],
        #     [0.0010, 0.9976, 0.0023],
        #     [-345.6516, -503.8226, 476.9076]
        # ])
        T = np.array([
            [0.0010, -0.000, 0.00],
            [-0.0000, 0.0010, -0.0000],
            [-0.2945, -0.3194, 0.5064]
        ])
        X = np.array([points[0], points[1], 1])
        U = np.dot(X, T)
        U[0] = U[0] / U[2]
        U[1] = U[1] / U[2]
        point = [U[0], U[1]]
        print('[DEBUG] x: ',point[0],'y: ',point[1])
        return point

    def stop(self):
        self._called_step_already = True
        self._checked_poses_already = False
        for k in self.robot_ip:
            data = {
                'action': 'set_velocity',
                'angular_speed': 0.0,
                'ip': self.robot_ip[k],
                'linear_speed': 0.0,
                'robot_id': k
            }
            data = json.dumps(data)
            self.ros_msg.publish(data)

    def stop_all(self):
        for i in range(100):
            self.stop()
            time.sleep(0.033)

    def get_real_position(self):
        msg = self.ros_sub
        delta = -0.0097
        while len(msg) == 0:
            msg = self.ros_sub
        tmp = msg
        ids = list(tmp.keys())
        for i in range(len(ids)):
            robot_id = ids[i]
            x = tmp[robot_id]['x']
            y = tmp[robot_id]['y']
            angle = tmp[robot_id]['angle']
            print('[DEBUG] ID: ', robot_id)
            location = self.pointsToWorld([x, y])
            for j in range(self.number_of_robots):
                if self.tf_id[j] == robot_id:
                    self.poses[0, j] = location[0] - math.cos(angle) * delta
                    self.poses[1, j] = location[1] - math.sin(angle) * delta
                    self.poses[2, j] = angle
        # logger.info("poses: " + self.poses)
        return self.poses



# ########################### YY #########################################

    def get_static_obs_position(self):
        msg = self.ros_sub
        delta = -0.0097
        while len(msg) == 0:
            msg = self.ros_sub
        tmp = msg
        ids = list(tmp.keys())
        # marker(id) of static obs is 200, 201, ...
        # marker(id) of static obs is 12, 13, 14, 15, 16, 17
        obs_pos_ls = []
        for i in range(len(ids)):
            obs_id = ids[i]
            print(obs_id)
            if int(obs_id.split('_')[-1]) < 12 or int(obs_id.split('_')[-1]) > 17:
                continue
            x = tmp[obs_id]['x']
            y = tmp[obs_id]['y']
            # angle = tmp[obs_id]['angle']
            print('[INFO] Static obs ID: ', obs_id)
            location = self.pointsToWorld([x, y])
            obs_pos_ls.append(location)

            # for j in range(self.number_of_robots):
            #     if self.tf_id[j] == robot_id:
            #         self.poses[0, j] = location[0] - math.cos(angle) * delta
            #         self.poses[1, j] = location[1] - math.sin(angle) * delta
            #         self.poses[2, j] = angle
        # logger.info("poses: " + self.poses)
        return obs_pos_ls

# ########################### YY #########################################

    def robot_id_tf(self, debug):
        if debug:
            for i in range(self.number_of_robots):
                self.tf_id[i] = i
        else:
            msg = self.ros_sub
            while len(msg) == 0:
                msg = self.ros_sub
            tmp = msg
            ids = list(tmp.keys())
            
            for i in range(self.number_of_robots):
                self.tf_id[i] = ids[i]

    def step_real(self):
        """Increments the simulation by updating the dynamics.
        """
        assert (not self._called_step_already), "Make sure to call get_poses before calling step() again."

        # Allow get_poses function to be called again.
        # Validate before thresholding velocities
        self._errors = self._validate()
        self._iterations += 1

        msg = self.ros_sub
        delta = -0.0097
        while len(msg) == 0:
            msg = self.ros_sub
        tmp = msg
        ids = list(tmp.keys())
        count = 0
        for i in range(self.number_of_robots):
            self._called_step_already = True
            self._checked_poses_already = False
            if self.tf_id[i] == ids[count]:
                if self.velocities[0, i] < 0:
                    linear_speed = 0.0
                else:
                    linear_speed = self.velocities[0, i]
                if abs(self.velocities[1, i]) < 0.000001:
                    angular_speed = 0.0
                else:
                    angular_speed = -self.velocities[1, i]
                count += 1
                if count > len(ids)-1:
                    count = len(ids)-1
            else:
                # if self.velocities[0, i] < 0:
                #     linear_speed = 0.0
                # else:
                linear_speed = self.velocities[0, i]
                if abs(self.velocities[1, i]) < 0.000001:
                    angular_speed = 0.0
                else:
                    angular_speed = -self.velocities[1, i]
            data = {
                'action': 'set_velocity',
                'angular_speed': angular_speed,
                'ip': self.robot_ip[self.tf_id[i]],
                'linear_speed': linear_speed,
                'robot_id': self.tf_id[i]
            }
            data = json.dumps(data)
            self.ros_msg.publish(data)
        # for i in range(len(ids)):
        #     self._called_step_already = True
        #     self._checked_poses_already = False
        #     robot_id = ids[i]
        #     for j in range(self.number_of_robots):
        #         if self.tf_id[j] == robot_id:
        #             if self.velocities[0, j] < 0:
        #                 linear_speed = 0.0
        #             else:
        #                 linear_speed = self.velocities[0, j]
        #             if abs(self.velocities[1, j]) < 0.000001:
        #                 angular_speed = 0.0
        #             else:
        #                 angular_speed = -self.velocities[1, j]
        #             break
        #     data = {
        #         'action': 'set_velocity',
        #         'angular_speed': angular_speed,
        #         'ip': self.robot_ip[robot_id],
        #         'linear_speed': linear_speed,
        #         'robot_id': robot_id
        #     }
        #     print(data)
        #     data = json.dumps(data)
        #     self.ros_msg.publish(data)

    def find_obstacle(self):
        obstacle = self.points_list
        return obstacle