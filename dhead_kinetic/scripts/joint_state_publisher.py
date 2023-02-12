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

    def catch_joint(self, Data):
        data = Data.data
        r = data[5] if data[5] < 180 else data[5] - 360
        p = data[3] if data[3] < 180 else data[3] - 360
        y = data[4] if data[4] < 180 else data[4] - 360
        fe = data[0]
        z = data[1] / 20000
        lb = data[2]
        self.joint_state.position = [-fe * pi / 180,
                                     lb * pi / 180,
                                     z / 20,
                                     y * pi / 180,
                                     -p * pi / 180,
                                     -r * pi / 180]
        self.joint_state.header.stamp = rospy.Time.now()
        self.pub.publish(self.joint_state)


if __name__ == '__main__':
    try:
        m_joint_catcher = joint_catcher()
    except rospy.ROSInterruptException:
        pass
