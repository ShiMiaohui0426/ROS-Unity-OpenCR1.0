#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String, Float32MultiArray
import threading

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


class joint_catcher():
    def __init__(self):
        super(joint_catcher, self).__init__()
        rospy.init_node("dhead_joint_states_publisher", anonymous=True)
        pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        sub = rospy.Subscriber("/head_command", Float32MultiArray, self.catch_joint)
        self.sub = sub
        self.pub = pub
        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.joint_state.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.joint_state.velocity = []
        self.joint_state.effort = []
        sub_thread = threading.Thread(target=rospy.spin)
        self.sub_trd = sub_thread
        self.sub_trd.start()
        print('start catch joint form HMD')

    def catch_joint(self, Data):
        data = Data.data
        r = data[5] if data[5] < 180 else data[5] - 360
        p = data[3] if data[3] < 180 else data[3] - 360
        y = data[4] if data[4] < 180 else data[4] - 360
        fe = data[0]
        z = data[1] / 20000
        lb = data[2]
        joint_goal = [0, 1, 2, 3, 4, 5]
        joint_goal[0] = fe * pi / 180
        joint_goal[1] = lb * pi / 180
        joint_goal[2] = z / 20
        joint_goal[3] = y * pi / 180
        joint_goal[4] = -p * pi / 180
        joint_goal[5] = -r * pi / 180
        self.joint_state.position = [joint_goal[0], joint_goal[1], joint_goal[2], joint_goal[3], joint_goal[4],
                                     joint_goal[5]]
        self.joint_state.header.stamp = rospy.Time.now()
        self.pub.publish(self.joint_state)


if __name__ == '__main__':
    m_joint_catcher = joint_catcher()
    '''
    if False:
        i = 0
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            i = i + 0.01

            m_joint_catcher.joint_state.position = [0.5,
                                                    0.5,
                                                    0.01,
                                                    0.5,
                                                    0.5,
                                                    i]
            m_joint_catcher.joint_state.header.stamp = rospy.Time.now()
            m_joint_catcher.pub.publish(m_joint_catcher.joint_state)
            #print(m_joint_catcher.joint_state.position)
            r.sleep()
    '''
