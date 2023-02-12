import tf2_ros
import tf
import rospy
import threading
from tf2_geometry_msgs import tf2_geometry_msgs

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

