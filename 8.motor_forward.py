import time
from pymavlink import mavutil

'''
Pixhawk内部的通道PWM范围是1000-2000us(即1ms-2ms)

PWM输出端口端口共14个,分别是1-14号通道
rc1~8分别对应飞控的main out 1~8
rc9~14分别对应aux out 1~6。
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

def set_rc_channel_pwm(channel_id,pwm_out=1500):
    if channel_id < 1 or channel_id > 14:
        raise ValueError("Channel ID must be between 1 and 14")
        return
    
    set_rc_channel_pwm = [65535 for _ in range(14)]
    set_rc_channel_pwm[channel_id - 1] = pwm_out
    
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *set_rc_channel_pwm
    )
    print(f"Set RC channel {channel_id} PWM to {pwm_out}\n\t")
   
   
    
if __name__ == "__main__":
    
    test_channel = 2  # 测试通道
    
    forward_pwm = 2000  # 向前的PWM值
    stop_pwm = 1500  # 停止的PWM值
    backward_pwm = 1000  # 向后的PWM值
    
    connection_string = 'udp:192.168.2.1:14550'  # 修改为你的连接地址
    master = connect_to_vehicle(connection_string)
    
    try:
        arm_vehicle(master)
        time.sleep(2)  # 等待2秒以确保飞控稳定
        
        # 设置PWM为前进模式
        print("motor forward!!\n\t")
        set_rc_channel_pwm(test_channel,forward_pwm)
        time.sleep(2)  # 保持前进模式5秒
        
        # 设置PWM为后退模式
        print("motor backward!!\n\t")
        set_rc_channel_pwm(test_channel,backward_pwm)
        time.sleep(2)  # 保持后退模式5秒
        
        # 设置PWM为停止模式
        print("motor stop!!\n\t")
        set_rc_channel_pwm(test_channel,stop_pwm)
        time.sleep(2)  # 保持停止模式5秒
        disarm_vehicle(master)
    

    except KeyboardInterrupt:
        disarm_vehicle(master)
        print("Vehicle disarmed")