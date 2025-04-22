import time
from pymavlink import mavutil

'''
bluerov2 电机布局如下
    电机2:左上(正桨)    电机1:右上(正桨)
    
    电机6:左中(反桨)    电机5:右中(正桨)
    
    电机4:左下(反桨)    电机3:右下(反桨)
'''
def connect_to_vehicle(connection_string):
    """连接飞控"""
    print(f"Connecting to vehicle on {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Heartbeat received. Connected to vehicle.")
    return master

def arm_vehicle(master):
    """解锁飞控"""
    print("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle is armed.\n\t")

def disarm_vehicle(master):
    """锁定飞控"""
    print("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Vehicle is disarmed.\n\t")


def GamePad_Simu(pad_x,pad_y,pad_z,pad_r,pad_states):
    print("GamePad_Input:")
    master.mav.manual_control_send(
        master.target_system,
        master.target_component,
        x=pad_x,
        y=pad_y, 
        z=pad_z, 
        r=pad_r,
        buttons=pad_states
    )
    

    
if __name__ == "__main__":
    

    connection_string = 'udp:192.168.2.1:14550'  # 修改为你的连接地址
    master = connect_to_vehicle(connection_string)
    
    try:
        arm_vehicle(master)
        time.sleep(2)  # 等待2秒以确保飞控稳定
        
    
        time.sleep(2)  # 保持停止模式5秒
        disarm_vehicle(master)
    

    except KeyboardInterrupt:
        disarm_vehicle(master)
        print("Vehicle disarmed")