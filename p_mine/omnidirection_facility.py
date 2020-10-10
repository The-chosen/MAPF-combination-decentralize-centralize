#!/usr/bin/env python3

import serial
import time
import json
import threading
from binascii import *
from crcmod import *
import socket
import math
import numpy as np

import rospy
from std_msgs.msg import String
from facility.msg import Box

class OmnidirectionFacility():
    def __init__(self):

        self.velocities = np.zeros((1,33))
        self.velocities = self.velocities.astype('int32')
        self.R = 0.095
        # self.origin = [0.223, 0.521] # RGB
        self.origin = [0.48, -0.282] # Realsense 
        self.max_linear_velocity = 150
        self.offset = 0.295
        self.ros_sub = {}
        self.box_info = Box()
        self._called_step_already = True
        self._checked_poses_already = False
        self.tf_id = {}

        self.poses = np.zeros((3,1))

        self.ser = serial.Serial("/dev/ttyUSB0", 115200)
        rospy.init_node('pios_topic_conveyor_node', anonymous=True)
        # rospy.Subscriber('/pios_topic_robot_pos', String, self.ros_callback)
        rospy.Subscriber('/pios_topic_box_pos', Box, self.ros_callback_contour)

    def ros_callback(self, msg):
        self.ros_sub = json.loads(msg.data)
    
    def ros_callback_contour(self, msg):
        self.box_info = msg

    def pointsToWorld(self, points):
        T = np.array([
            [1.0012, -0.0009, -0.0017],
            [0.0010, 0.9976, 0.0023],
            [-345.6516, -503.8226, 476.9076]
        ])
        X = np.array([points[0], points[1], 1])
        U = np.dot(X, T)
        U[0] = U[0] / U[2]
        U[1] = U[1] / U[2]
        point = [U[0], U[1]]
        return point

    def get_robot_id(self, debug):
        if debug == 0:
            msg = self.ros_sub
            while len(msg) == 0:
                msg = self.ros_sub
            tmp = msg
            ids = list(tmp.keys())
            for i in range(len(ids)):
                robot_id = ids[i]
        else:
            robot_id = 0
        return robot_id

    def get_map(self):
        """Returns the states of all rollers
        based on initial position
        """
        roller_map = []
        unit_map = []
        unit_x = self.origin[0]
        unit_y = self.origin[1]
        id = 0
        for i in range(3):
            for j in range(4):
                unit_map.append(np.array([[unit_x],[unit_y]]))
                if i == 1 and j == 0:
                    # unit_x = self.origin[0]+self.offset/2
                    unit_x = self.origin[0]-self.offset/2
                    continue
                for k in range(3):
                    id +=1
                    # angle = np.pi + np.pi*2/3*k
                    angle = np.pi*2/3*k
                    angle = np.arctan2(np.sin(angle),np.cos(angle))
                    x = unit_x + self.R*np.cos(angle)
                    y = unit_y + self.R*np.sin(angle)
                    # x = unit_x - self.R*np.cos(angle)
                    # y = unit_y - self.R*np.sin(angle)
                    
                    roller_map.append(Roller(x,y,angle,id))
                # unit_x += self.offset
                unit_x -= self.offset
                
            unit_x = self.origin[0]
            # unit_y -= self.offset*np.sin(np.pi/3)
            unit_y += self.offset*np.sin(np.pi/3)
        return roller_map, unit_map

    def get_roller_id(self):
        x_min,x_max,y_min,y_max = 10,-10,10,-10

        center_x = self.box_info.center[0]
        center_y = self.box_info.center[1]
        angle = self.box_info.angle

        for i in range(4):
            temp_x = -(self.box_info.vertices[i*2]-center_x)*np.cos(angle)+(self.box_info.vertices[i*2+1]-center_y)*np.sin(angle)
            temp_y = -(self.box_info.vertices[i*2]-center_x)*np.sin(angle)-(self.box_info.vertices[i*2+1]-center_y)*np.cos(angle)
            if temp_x < x_min:
                x_min = temp_x
            if temp_x > x_max:
                x_max = temp_x
            if temp_y < y_min:
                y_min = temp_y
            if temp_y > y_max:
                y_max = temp_y
        indexes = []
        index = np.array([])
        index = index.astype('int32')
        roller_map,_ = self.get_map()
        padding = 0.05 
        for roller in roller_map:
            x_new = -(roller.x - center_x)*np.cos(angle)+(roller.y - center_y)*np.sin(angle)
            y_new = -(roller.x - center_x)*np.sin(angle)-(roller.y - center_y)*np.cos(angle)
            if (x_min-padding) <= x_new <= (x_max+padding) and (y_min-padding) <= y_new <= (y_max+padding):
                index = np.append(index,np.array([roller.id-1]))
        if index.size != 0:
            indexes.append(index)
        # print(indexes)
        return indexes

    def set_velocities(self, roller_map):
        for i in range(len(self.velocities[0])):
            vel = int(roller_map[i].velocity*1000)
            if vel > 300:
                self.velocities[0][i] = 300
            elif 0 < vel < 20:
                self.velocities[0][i] = 20
            elif vel < -200:
                self.velocities[0][i] = -200
            elif -20 < vel < 0:
                self.velocities[0][i] = -20
            else:
                self.velocities[0][i] = vel


    def get_real_position(self):
        msg = self.ros_sub
        delta = 0
        while len(msg) == 0:
            msg = self.ros_sub
        tmp = msg
        ids = list(tmp.keys())
        self._called_step_already = False
        self._checked_poses_already = True
        for i in range(3):
            self.poses[0, i] = -1
            self.poses[1, i] = -1
            self.poses[2, i] = 0

        for i in range(len(ids)):
            robot_id = ids[i]
            x = tmp[robot_id]['x']
            y = tmp[robot_id]['y']
            angle = tmp[robot_id]['angle']
            location = self.pointsToWorld([x, y])
            world_x = location[0] - math.cos(angle) * delta
            world_y = location[1] - math.sin(angle) * delta
            x_min = self.origin[0]-0.8*self.offset
            x_max = self.origin[0] + 3.5*self.offset
            y_max = self.origin[1]+1.5*self.offset
            y_min = self.origin[1] - 2 * self.offset
            if x_min < world_x < x_max and y_min < world_y < y_max:
                self.poses[0, i] = world_x
                self.poses[1, i] = world_y
                self.poses[2, i] = -angle
            # self.poses[0, i] = location[0] - math.cos(angle) * delta
            # self.poses[1, i] = location[1] - math.sin(angle) * delta
            # self.poses[2, i] = -angle
            # for j in range(self.number_of_robots):
        return self.poses

    def get_real_position_from_contour(self):
        while len(self.box_info.box_id.data) == 0:
            print("Waiting for box info")
            time.sleep(0.2)
        ids = self.box_info.box_id
        self._called_step_already = False
        self._checked_poses_already = True

        self.poses[0] = self.box_info.center[0]
        self.poses[1] = self.box_info.center[1]
        self.poses[2] = self.box_info.angle

        return self.poses

    def step_real(self):
        """Updating the system."""

        assert (not self._called_step_already), "Make sure to call get_poses before calling step() again."

        msg = self.ros_sub
        while len(msg) == 0:
            msg = self.ros_sub
        tmp = msg
        ids = list(tmp.keys())

        self._called_step_already = True
        self._checked_poses_already = False
        self.send_command(self.velocities)

    def step_real2(self):
        """Updating the system."""

        assert (not self._called_step_already), "Make sure to call get_poses before calling step() again."
        self._called_step_already = True
        self._checked_poses_already = False
        self.send_command(self.velocities)

    def step(self,dxi):
        """Increments the simulation by updating the dynamics.
        """
        assert (not self._called_step_already), "Make sure to call get_poses before calling step() again."

        # Allow get_poses function to be called again.
        self._called_step_already = True
        self._checked_poses_already = False

        # Validate before thresholding velocities
        # self._errors = self._validate()
        # self._iterations += 1
        idxs = np.where(np.abs(dxi[:2]) > 0.1)
        dxi[idxs] = 0.1*np.sign(dxi[idxs])
        # Update dynamics of agents
        self.poses[0, :] = self.poses[0, :] + self.time_step * dxi[0]
        self.poses[1, :] = self.poses[1, :] + self.time_step * dxi[1]
        self.poses[2, :] = self.poses[2, :] + self.time_step * dxi[2]
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

    def crc16Add(self, read):
        crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
        data = read.replace(" ", "")
        readcrcout = hex(crc16(unhexlify(data))).upper()
        str_list = list(readcrcout)
        crc_data = "".join(str_list)[2:].zfill(4)
        read = read.strip() + ' ' + crc_data[2:] + ' ' + crc_data[0:2]
        return read

    def toHex(self, num):
        chaDic = {10: 'a', 11: 'b', 12: 'c', 13: 'd', 14: 'e', 15: 'f'}
        hexStr = ""
        if num < 0:
            num = num + 2 ** 16
        while num >= 16:
            digit = num % 16
            hexStr = chaDic.get(digit, str(digit)) + hexStr
            num //= 16
        hexStr = str(chaDic.get(num, str(num)) + hexStr).zfill(4)
        res = hexStr[0:2] + ' ' + hexStr[2:4]
        return res

    def send_command(self,velocities):
        # print("velocity push:",velocities)
        try:
            msg = "70 69 6F 73 4F 01 00 00 42 "
            RollerVelocity = ""
            for velocity in velocities[0]:
                RollerVelocity += self.toHex(velocity)+" "
            CrcInput = msg + RollerVelocity
            send = self.crc16Add(CrcInput) + " " + "AF ED"
            bytes_msg = bytes.fromhex(send)
            self.ser.write(bytes_msg)
            time.sleep(0.02)
        except:
            pass

    def recviceMsg(self):
        
        while True:
            try:
                msg = "70 69 6F 73 0E 01 00 01 01 01 19 AF AF ED "
                bytes_msg = bytes.fromhex(msg)
                self.ser.write(bytes_msg)
                time.sleep(0.02)
                count = self.ser.inWaiting()
                if count == 79:
                    recv = self.ser.read(count)
                    print("recv",recv)
                    MsgHex = ''.join(['%02x' % b for b in recv])
                    crc = self.crc16Add(MsgHex[:-8]) + " " + "AF ED"
                    bytes_msg = bytes.fromhex(crc)
                    if bytes_msg == recv:
                        print("========================")
                        # left_velocity = self.hex_dec(MsgHex[20:22]+MsgHex[18:20])
                        # if left_velocity > 32768:
                        #     left_velocity = left_velocity - 65535
                        # right_velocity = self.hex_dec(MsgHex[24:26]+MsgHex[22:24])
                        # if right_velocity > 32768:
                        #     right_velocity = right_velocity - 65535
                        # battery = self.hex_dec(MsgHex[26:28])
                        # print("battery:", MsgHex[26:28])
                        # self.l = float(left_velocity)
                        # self.r = float(right_velocity)
                self.ser.flushInput()
                time.sleep(0.02)
            except:
                pass

class Roller():
    def __init__(self, x = 0, y = 0, angle = 0, id = 0,velocity=0):
        self.x = x
        self.y = y
        self.id = id
        self.angle = angle
        self.velocity = velocity