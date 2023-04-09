#!/usr/bin/env python
from pymycobot import MyCobotSocket
import time

def wait_moving(robot):
    while robot.is_moving():
        print('moving')
        time.sleep(0.5)


# mc = MyCobot("/dev/ttyACM1", 115200)
mc = MyCobotSocket("192.168.1.3", 9000)
mc.send_angles([0, 0, 0, 0, 0, 0], 50)
wait_moving(mc)
mc.send_angles([-90, -30, -45, -15, 0, 0], 50)
wait_moving(mc)
coords = mc.get_coords()
print(coords)
# robot.send_coords([-50, -221.9, 202.3, 178.5, 0.26, -179.91], 80, 1)

coords = mc.get_coords()
print(coords)
mc.set_encoder(7, 2000)

