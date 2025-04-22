
"""
用pymavlink协议去解锁和锁上飞行器
"""

# 解锁

import time
from pymavlink import mavutil
from time import sleep

def arm():
    master = mavutil.mavlink_connection('udp:192.168.2.1:{}'.format(14550))
    master.wait_heartbeat()
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    print("Waiting for the vehicle to arm")

    print('Armed!')

# 上锁
def disarm():
    master = mavutil.mavlink_connection('udp:192.168.2.1:{}'.format(14550))
    master.wait_heartbeat()
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    master.motors_disarmed_wait()
    print("Waiting for the vehicle to disarm")
    print('Disarmed!')  
    
while True:
    disarm()
    sleep(5)
    # 间隔5秒钟