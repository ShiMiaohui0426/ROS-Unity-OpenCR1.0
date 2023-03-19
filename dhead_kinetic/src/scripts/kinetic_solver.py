#!/usr/bin/env python
from dhead_pose_tf_sub import dhead_pose_tf_Subscriber
from kinetic_tcp_srv import kinect_tcp_server
import json
from fake_robot import fake_robot

robot = fake_robot()


class kinetic_server:
    def __init__(self):
        self.data = None
        self.target=None
        self.case = {
            'wait_connect': self.wait_connect,
            'response': self.response,
        }

        self.serves = {
            'set_grasp_target':self.set_grasp_target,
            'start_grasp': self.start_grasp,
            'query_grasp_done': self.query_grasp_done,
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
        cam_target=self.data['position']
        pos = self.tfsb.slove_object_pose([cam_target['x'], cam_target['y'], cam_target['z']])
        self.target=pos

    def start_grasp(self):
        print('start grasp')
        robot.start_grasp(self.target)

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
