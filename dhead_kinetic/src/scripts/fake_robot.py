#!/usr/bin/env python
import time
from pymycobot.mycobot import MyCobot
from pymycobot import MyCobotSocket
import threading
import math



#旋转矩阵转四元数
def rotation_matrix_to_quaternion(matrix):
    qw = math.sqrt(1 + matrix[0][0] + matrix[1][1] + matrix[2][2]) / 2
    qx = (matrix[2][1] - matrix[1][2]) / (4 * qw)
    qy = (matrix[0][2] - matrix[2][0]) / (4 * qw)
    qz = (matrix[1][0] - matrix[0][1]) / (4 * qw)
    return [qx, qy, qz, qw]

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
        self.target = [pos[0] * 1000 - 475, pos[1] * 1000 - 125, pos[2] * 1000]
        if self.target[2] < 160:
            self.target[2] = 165
        self.busy = True
        print('grasp at', self.target)

    def GripperCatch(self):
        self.mc.set_encoder(7, 0)
        time.sleep(2)

    def GripperRelease(self):
        self.mc.set_encoder(7, 2000)
        time.sleep(2)

    def GripperMoveToDestination(self):
        self.mc.send_angles([90, -45, 0, 0, 0, 0], 60)
        wait_moving(self.mc)

    def GripperMoveToOrigin(self):
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 80)
        wait_moving(self.mc)

    def GripperMoveTo(self, tar):
        self.mc.send_coords(tar, speed, 0)
        wait_moving(self.mc)

    def grasp_thread_task(self):
        while True:
            if self.busy:
                self.GripperMoveToOrigin()
                m_target = [self.target[0], self.target[1], self.target[2], -179, -1, -179]
                topoftarget = [self.target[0], self.target[1], self.target[2] + 120, -179, -1, -179]
                self.GripperMoveTo(topoftarget)

                topoftarget = [self.target[0], self.target[1], self.target[2] + 110, -179, -1,
                               -179 - 10]
                self.GripperMoveTo(topoftarget)
                topoftarget = [self.target[0], self.target[1], self.target[2] + 100, -179, -1,
                               -179 + 10]
                self.GripperMoveTo(topoftarget)
                for i in range(1, 3):
                    topoftarget = [self.target[0], self.target[1], self.target[2] + 100 - 50 * i, -179, -1,
                                   -179]
                    self.GripperMoveTo(topoftarget)
                    topoftarget = [self.target[0], self.target[1], self.target[2] + 100 - 50 * i, -179, -1,
                                   -179 - 10*i]
                    self.GripperMoveTo(topoftarget)
                    topoftarget = [self.target[0], self.target[1], self.target[2] + 100 - 50 * i, -179, -1,
                                   -179 + 10*i]
                    self.GripperMoveTo(topoftarget)

                print('move to the top of target')
                self.GripperMoveTo(topoftarget)
                topoftarget = [self.target[0], self.target[1], self.target[2] + 50, -179, -1, -179]
                print('move to the target')
                self.GripperMoveTo(m_target)
                print('pick the target')
                self.GripperCatch()
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
