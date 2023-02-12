#!/usr/bin/env python

import tf2_ros
import tf
import rospy
import moveit_commander
import moveit_msgs.msg
import sys

from sensor_msgs.msg import JointState

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

from geometry_msgs.msg import TransformStamped
import threading

class dhead_pose_tf_publisher():
    def __init__(self):
        super(dhead_pose_tf_publisher, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("dhead_pose_tf_publisher", anonymous=True)
        sub = rospy.Subscriber('/joint_states', JointState, self.send_pose_tf)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "dhead"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        tfpub = tf2_ros.StaticTransformBroadcaster()  # 创建发布对象

        self.tfpub = tfpub

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group

        move_group.set_max_acceleration_scaling_factor(1)
        move_group.set_max_velocity_scaling_factor(1)

        sub_thread = threading.Thread(target=rospy.spin)
        self.sub_trd = sub_thread
        self.sub_trd.start()


    def send_pose_tf(self,data):
        wpose = self.move_group.get_current_pose().pose
        ts = TransformStamped()  # 组织被发布的数据

        # header
        ts.header.stamp = rospy.Time.now()  # 时间戳
        ts.header.frame_id = "base_link"  # 父坐标系

        # child frame
        ts.child_frame_id = "dhead"  # 子坐标系

        ts.transform.translation.x = wpose.position.x
        ts.transform.translation.y = wpose.position.y
        ts.transform.translation.z = wpose.position.z

        ts.transform.rotation.x = wpose.orientation.x
        ts.transform.rotation.y = wpose.orientation.y
        ts.transform.rotation.z = wpose.orientation.z
        ts.transform.rotation.w = wpose.orientation.w
        # 发布数据
        self.tfpub.sendTransform(ts)

if __name__ == '__main__':
    try:
        m_dhead_pose_tf_publisher = dhead_pose_tf_publisher()
    except rospy.ROSInterruptException:
        pass