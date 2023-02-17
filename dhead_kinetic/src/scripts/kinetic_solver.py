#!/usr/bin/env python
from dhead_pose_tf_sub import dhead_pose_tf_Subscriber
from kinetic_tcp_srv import kinect_tcp_server
import json
from fake_robot import fake_robot

robot = fake_robot()


class state_machine:
    def __init__(self):
        self.jdata = None
        self.data = None
        self.case = {
            'wait_connect': self.wait_connect,
            'wait_command': self.wait_command,
            'start_grasp': self.start_grasp,
            'wait_grasp_done': self.wait_grasp_done
        }
        self.srv = kinect_tcp_server()
        self.state = 0
        self.tfsb = dhead_pose_tf_Subscriber()

    def switch_auto(self):
        method = self.case.get(self.state)
        if method:
            method()

    def wait_connect(self):
        self.srv.wait_connect()
        self.state = 'wait_command'

    def wait_command(self):
        try:
            if not self.srv.empty_input():
                self.data = self.srv.get_data()
                self.jdata = json.loads(self.data)
                self.state = self.data['command']
        except:
            self.srv.close_connect()
            print('Kinetic closed unexpectedly')

    def start_grasp(self):
        position = self.jdata['position']
        pos = self.tfsb.slove_object_pose(position['x'], position['y'], position['z'])
        robot.start_grasp(pos)
        self.state = 'wait_grasp_done'

    def wait_grasp_done(self):
        if not robot.is_busy():
            self.state = 'wait_command'
        data = {
            'command': 'grasp_done'
        }
        str_json = json.dumps(data)
        self.srv.send_json(str_json)


def main():
    print('start kinctic solver')
    m_state_machine = state_machine()
    while True:
        m_state_machine.switch_auto()


if __name__ == "__main__":
    main()
