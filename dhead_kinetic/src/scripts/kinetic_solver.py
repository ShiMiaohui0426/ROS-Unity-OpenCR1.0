#!/usr/bin/env python
from dhead_pose_tf_sub import dhead_pose_tf_Subscriber
from kinetic_tcp_srv import kinect_tcp_server
import json
from fake_robot import fake_robot

robot = fake_robot()

import numpy as np


def med_filter(data, n):
    N = len(data)
    m = int((n - 1) / 2)
    for i in range(0, m):
        data[i] = np.median(data[0:n])

    for i in range(m, N - m):
        data[i] = np.median(data[i - m:i + m + 1])

    for i in range(0, m):
        data[N - i - 1] = np.median(data[N - n:N])
    return data


class kinetic_server:
    def __init__(self):
        self.data = None
        self.target = None
        self.case = {
            'wait_connect': self.wait_connect,
            'response': self.response,
        }

        self.serves = {
            'set_grasp_target': self.set_grasp_target,
            'clear_grasp_target': self.clear_grasp_target,
            'start_grasp': self.start_grasp,
            'query_grasp_done': self.query_grasp_done,
        }
        self.srv = kinect_tcp_server()
        self.state = 0
        self.tfsb = dhead_pose_tf_Subscriber()
        self.target_list = []

    def switch_auto(self):
        method = self.case.get(self.state)
        if method:
            method()

    def wait_connect(self):
        self.srv.wait_connect()
        self.state = 'response'

    def response(self):
        try:
            if not self.srv.empty_input():
                self.data = self.srv.get_data()
                method = self.serves.get(self.data['command'])
                if method:
                    print('response to ', self.data['command'])
                    method()

                else:
                    print('receive: ', self.data)


        except Exception as message:
            print('error: %s' % message)
            print('Kinetic closed unexpectedly')

    def set_grasp_target(self):
        cam_target = self.data['position']
        pos = self.tfsb.slove_object_pose([cam_target['x'], cam_target['y'], cam_target['z']])
        if pos:
            self.target_list.append(pos)

    def clear_grasp_target(self):
        self.target_list = []

    def start_grasp(self):
        x_list = []
        y_list = []
        z_list = []
        print('calaulate point')
        for i in self.target_list:
            x_list.append(i[0])
            y_list.append(i[1])
            z_list.append(i[2])
        print('calaulate point1')
        x_list = med_filter(x_list, 7)
        y_list = med_filter(y_list, 7)
        z_list = med_filter(z_list, 7)
        print('calaulate point done')
        pos = [np.mean(x_list), np.mean(y_list), np.mean(z_list)]
        print('start grasp')
        self.target = pos
        robot.start_grasp(self.target)
        self.target_list = []

    def query_grasp_done(self):
        if not robot.is_busy():
            data = {'command': 'grasp_done'}
            str_json = json.dumps(data)
            self.srv.send_json(str_json)
            print('grasp_done')
        else:
            print('robot busy')


def main():
    print('start kinctic solver')
    m_state_machine = kinetic_server()
    m_state_machine.wait_connect()
    while True:
        m_state_machine.switch_auto()


if __name__ == "__main__":
    main()
