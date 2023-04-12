#!/usr/bin/env python
import tf2_ros
import tf
import rospy
import threading
from tf2_geometry_msgs import tf2_geometry_msgs
import math

#四元数转角度欧拉角
def quaternion_to_euler(x, y, z, w):
    ysqr = y * y
    t0 = -2.0 * (ysqr + z * z) + 1.0
    t1 = +2.0 * (x * y + w * z)
    t2 = -2.0 * (x * z - w * y)
    t3 = +2.0 * (y * z + w * x)
    t4 = -2.0 * (x * x + ysqr) + 1.0
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    roll = math.atan2(t3, t4)
    pitch = math.asin(t2)
    yaw = math.atan2(t1, t0)
    return [roll, pitch, yaw]

class dhead_pose_tf_Subscriber():
    def __init__(self):
        super(dhead_pose_tf_Subscriber, self).__init__()
        rospy.init_node("dhead_pose_tf_subscriber", anonymous=True)
        self.tfbuffer = tf2_ros.Buffer()  # 创建缓存对象
        self.tfsub = tf2_ros.TransformListener(self.tfbuffer)  # 创建订阅对象，将缓存传入

        sub_thread = threading.Thread(target=rospy.spin)
        self.sub_trd = sub_thread
        self.sub_trd.start()

    def slove_object_pose(self, object_pose):
        ps = tf2_geometry_msgs.PoseStamped()  # 组织被转换的坐标点
        ps.header.stamp = rospy.Time.now()  # 时间戳
        ps.header.frame_id = "dhead"  # 参考坐标系
        ps.pose.position.x = object_pose[0]
        ps.pose.position.y = -object_pose[1]
        ps.pose.position.z = -object_pose[2]
        ps.pose.orientation.x = object_pose[3]
        ps.pose.orientation.y = object_pose[4]
        ps.pose.orientation.z = object_pose[5]
        ps.pose.orientation.w = object_pose[6]

        try:
            # 转换实现
            ps_out = self.tfbuffer.transform(ps, "base_link")
            rpy=quaternion_to_euler(ps_out.pose.orientation.x,ps_out.pose.orientation.y,ps_out.pose.orientation.z,ps_out.pose.orientation.w)

            # 输出结果
            rospy.loginfo("转换后的坐标：(%.2f, %.2f, %.2f, %2f), 参考的坐标系：%s",
                          ps_out.pose.position.x,
                          ps_out.pose.position.y,
                          ps_out.pose.position.z,
                          -rpy[2]*180/3.1415926,
                          ps_out.header.frame_id)
            with open('/root/Desktop/ROS-Unity-OpenCR1.0/dhead_kinetic/src/scripts/data.txt', 'a') as f:
                text = str(ps_out.pose.position.x) + ',' + str(ps_out.pose.position.y) + ',' + str(ps_out.pose.position.z) + ',' + str(rpy[2])+'\n\r'
                f.write(text)

            opose = [ps_out.pose.position.x,
                     ps_out.pose.position.y,
                     ps_out.pose.position.z,
                     rpy[0],
                     rpy[1],
                     -rpy[2]]

            return opose
        except Exception as e:
            rospy.loginfo("错误提示：%s", e)
