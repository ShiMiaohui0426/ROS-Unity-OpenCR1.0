#!/usr/bin/env python
from __future__ import print_function
from six.moves import input
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
from std_msgs.msg import String, Float32MultiArray
from moveit_commander.conversions import pose_to_list
import threading
import queue
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import tf2_geometry_msgs


class dhead_kinetic():
    def __init__(self):
        super(dhead_kinetic, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("dhead_kinetic", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "dhead"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher

        self.planning_frame = planning_frame
        self.group_names = group_names
        move_group.set_max_acceleration_scaling_factor(1)
        move_group.set_max_velocity_scaling_factor(1)
        wpose = move_group.get_current_pose().pose
        # print(wpose)
        self.pose = wpose
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        self.joint = joint_goal

        # wpose = move_group.get_current_pose().pose
        # print(wpose)

        self.thread_rosspin = threading.Thread(target=rospy.spin)

        self.poseQueue = queue.Queue(1024)
        self.getPose = False

        tfpub = tf2_ros.StaticTransformBroadcaster()  # 创建发布对象

        self.tfpub = tfpub
        self.tfbuffer = tf2_ros.Buffer()  # 创建缓存对象

        self.tfsub = tf2_ros.TransformListener(self.tfbuffer)  # 创建订阅对象，将缓存传入

        dhead_subscriber = rospy.Subscriber("/head_command", Float32MultiArray, self.dhead_info)
        self.dhead_subscriber = dhead_subscriber
        self.start_subscriber()
        self.setjoint = False
        self.thread_setjoint = threading.Thread(target=self.set_dhead_moveit_joint)
        self.thread_setjoint.start()

    def set_dhead_moveit_joint(self):
        while True:
            if self.setjoint:
                self.move_group.go(wait=True)
                self.move_group.stop()
                wpose = self.move_group.get_current_pose().pose
                self.poseQueue.put(wpose)
                print("set joint done")
                #print(self.joint)
                self.setjoint = False

    def get_dhead_moveit_pose(self):
        self.getPose = True
        while True:
            if not self.poseQueue.empty():
                wpose = self.poseQueue.get()
                self.pose = wpose
                # 相对关系(偏移量与四元数)
                print(self.pose)
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
                break

    def dhead_info(self, Data):
        data = Data.data
        joint_goal = self.joint
        r = data[5] if data[5] < 180 else data[5] - 360
        p = data[3] if data[3] < 180 else data[3] - 360
        y = data[4] if data[4] < 180 else data[4] - 360
        fe = data[0]
        z = data[1]/20000
        lb = data[2]
        joint_goal[0] = -fe*pi/180
        joint_goal[1] = lb*pi/180
        joint_goal[2] = z / 20
        joint_goal[3] = y*pi/180
        joint_goal[4] = -p*pi/180
        joint_goal[5] = -r*pi/180
        self.joint = joint_goal

        #print(joint_goal)
        if self.getPose:
            #print("start set joint")
            self.move_group.set_joint_value_target(self.joint)
            self.setjoint = True
            self.getPose = False
        # print(data)

    def start_subscriber(self):
        self.thread_rosspin.start()

    def slove_object_pose(self, object_pose):
        ps = tf2_geometry_msgs.PointStamped()  # 组织被转换的坐标点
        ps.header.stamp = rospy.Time.now()  # 时间戳
        ps.header.frame_id = "dhead"  # 参考坐标系
        ps.point.x = object_pose[0]  # 相机坐标点
        ps.point.y = object_pose[1]
        ps.point.z = object_pose[2]
        try:
            # 转换实现
            ps_out = self.tfbuffer.transform(ps, "base_link")

            # 输出结果
            rospy.loginfo("转换后的坐标：(%.2f, %.2f, %.2f), 参考的坐标系：%s",
                          ps_out.point.x,
                          ps_out.point.y,
                          ps_out.point.z,
                          ps_out.header.frame_id)
            qtn = tf.transformations.quaternion_from_euler(0, 0, 1)
            opose = [ps_out.point.x,
                     ps_out.point.y,
                     ps_out.point.z,
                     qtn[0],
                     qtn[1],
                     qtn[2],
                     qtn[3]]

            return opose
        except Exception as e:
            rospy.loginfo("错误提示：%s", e)


def main():
    kinetic = dhead_kinetic()
    while True:
        kinetic.get_dhead_moveit_pose()


if __name__ == "__main__":
    main()
