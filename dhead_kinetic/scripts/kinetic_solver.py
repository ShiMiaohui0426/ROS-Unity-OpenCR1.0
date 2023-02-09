from kinetic_moveit import dhead_kinetic
from kinetic_tcp_srv import kinect_tcp_server


def main():
    srv = kinect_tcp_server()
    srv.wait_connect()
    kinetic = dhead_kinetic()
    while True:
        try:
            if not srv.empty_input():
                data = srv.get_data()
                kinetic.get_dhead_moveit_pose()
                opose = kinetic.slove_object_pose([data[0], data[1], data[2]])
                srv.send_pose(opose)
        except:
            srv.close_connect()


if __name__ == "__main__":
    main()
