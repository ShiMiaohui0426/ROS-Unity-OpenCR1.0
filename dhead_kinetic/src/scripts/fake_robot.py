#!/usr/bin/env python
import time
from pymycobot.mycobot import MyCobot
from pymycobot import MyCobotSocket
import threading
import math


# 旋转矩阵转四元数


def wait_moving(robot):
    # time.sleep(1)
    while robot.is_moving():
        time.sleep(0.05)


speed = 20


class fake_robot:
    def __init__(self):
        self.start = None
        self.busy = False
        self.start = 0
        self.target = None
        # self.mc = MyCobot("/dev/ttyACM0", 115200)
        self.mc = MyCobotSocket("192.168.1.3", 9000)
        self.init_on = True
        self.grasp_thread = threading.Thread(target=self.grasp_thread_task)
        self.grasp_thread.start()

    def start_grasp(self, pos):
        self.target = [pos[0] * 1000 - 475, pos[1] * 1000 - 125, 165, pos[5] * 180 / 3.1415926]
        if self.target[2] < 160:
            self.target[2] = 165
        if 0 < self.target[3] < 90:
            self.target[3] -= 180
        '''
        if self.target[3] > 0:
            self.target[3] = self.target[3] - 180
        if self.target[3] < -180:
            self.target[3] = self.target[3] + 180
        '''
        self.busy = True
        print('grasp at', self.target)

    def GripperCatch(self):
        self.mc.set_encoder(7, 0)
        time.sleep(2)

    def GripperRelease(self):
        self.mc.set_encoder(7, 2200)
        time.sleep(2)

    def GripperMoveToDestination(self):
        self.mc.send_angles([90, -45, 0, 0, 0, 0], 60)
        wait_moving(self.mc)

    def GripperMoveToOrigin(self):
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 80)
        wait_moving(self.mc)

    def GripperMoveTo(self, tar, type=0):
        self.mc.send_coords(tar, speed, type)
        wait_moving(self.mc)

    def grasp_thread_task(self):
        while True:
            if self.busy:
                self.mc.send_angles([0, 0, 0, 0, 0, 0], 80)
                wait_moving(self.mc)
                a = self.target[3] - 90
                if a > 0:
                    a = a - 180
                if a < -180:
                    a = a + 180
                if a < -90:
                    a = a + 180

                print('grasp init at')
                print([a, -30, 0, 0, 0, 0])
                self.mc.send_angles([a, -30, 0, 0, 0, 0], 80)
                a = a - 90

                if a < -180:
                    a = a + 360
                wait_moving(self.mc)
                coords = self.mc.get_coords()
                coords[0] = self.target[0]
                coords[1] = self.target[1]
                coords[2] = coords[2] - 50
                # self.GripperMoveTo(coords)
                self.mc.send_coords(coords, 80, 0)
                wait_moving(self.mc)
                m_target = [self.target[0], self.target[1], self.target[2], -179, -1, a]
                topoftarget = [self.target[0], self.target[1], self.target[2] + 120, -179, -1, a]
                print('move to the top of target')
                self.GripperMoveTo(topoftarget, 1)
                '''
                for i in range(1, 3):
                    topoftarget = [self.target[0], self.target[1], self.target[2] + 100 - 50 * i, -179, -1,
                                   self.target[3]]
                    self.GripperMoveTo(topoftarget)
                    topoftarget = [self.target[0], self.target[1], self.target[2] + 100 - 50 * i, -179, -1,
                                   self.target[3] - 10 * i]
                    self.GripperMoveTo(topoftarget)
                    topoftarget = [self.target[0], self.target[1], self.target[2] + 100 - 50 * i, -179, -1,
                                   self.target[3] + 10 * i]
                    self.GripperMoveTo(topoftarget)
'''
                print('move to the target')
                self.GripperMoveTo(m_target, 1)
                print('pick the target')
                self.GripperCatch()

                topoftarget = [self.target[0], self.target[1], self.target[2] + 50, -179, -1, a]
                self.GripperMoveTo(topoftarget)

                print('move to the Destination')
                self.GripperMoveToOrigin()
                self.GripperMoveToDestination()
                print('pick the target')
                self.GripperRelease()
                self.busy = False
            if self.init_on:
                self.mc.send_angles([0, 0, 0, 0, 0, 0], speed)
                wait_moving(self.mc)
                self.mc.set_encoder(7, 2000)
                time.sleep(2)
                wait_moving(self.mc)
                self.mc.send_angles([90, -45, 0, 0, 0, 0], speed)
                wait_moving(self.mc)
                self.init_on = False

    def is_busy(self):
        return self.busy
