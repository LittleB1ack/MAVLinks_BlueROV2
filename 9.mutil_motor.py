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

def set_rc_channel_pwm(channel_id, pwm_out=1500):
    if channel_id < 1 or channel_id > 14:
        raise ValueError("Channel ID must be between 1 and 14")
        return
    if pwm_out < 1000 or pwm_out > 2000:
        raise ValueError("PWM value must be between 1000 and 2000")
        return

    set_rc_channel_pwm = [1500 for _ in range(14)]

    if pwm_out == 1500:
        # 停止模式
        set_rc_channel_pwm[channel_id - 1] = pwm_out

    elif pwm_out < 1500:
        # PWM 小于 1500
        print("pwm_out < 1500")
        if channel_id in (1, 2, 5):
            # 通道 1、2、5：正转
            print("forward! channel:1, 2, 5")
            set_rc_channel_pwm[channel_id - 1] = pwm_out
        elif channel_id in (3, 4, 6):
            # 通道 3、4、6：反转
            print("backward! channel:3, 4, 6")
            set_rc_channel_pwm[channel_id - 1] = 1500 + (1500 - pwm_out)

    elif pwm_out > 1500:
        # PWM 大于 1500
        print("pwm_out > 1500")
        if channel_id in (1, 2, 5):
            # 通道 1、2、5：反转
            print("backward! channel:1, 2, 5")
            set_rc_channel_pwm[channel_id - 1] = pwm_out
        elif channel_id in (3, 4, 6):
            # 通道 3、4、6：正转
            print("forward! channel:3, 4, 6")
            set_rc_channel_pwm[channel_id - 1] = 1500 - (pwm_out - 1500)
    
    print("-----------------------------------------------\n\t")        
    for i in range(14):
        print( f"Channel {i+1} PWM: {set_rc_channel_pwm[i]}")      
    print("-----------------------------------------------\n\t")      
    
    # 发送PWM值到飞控
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *set_rc_channel_pwm
    )

   
   
    
if __name__ == "__main__":
    
    test_channel = 2  # 测试通道

    
    forward_pwm = 1400  # 向前的PWM值
    stop_pwm = 1500  # 停止的PWM值
    backward_pwm = 1600  # 向后的PWM值
    
    error_pwm = 3000  # 错误的PWM值
    
    connection_string = 'udp:192.168.2.1:14550'  # 修改为你的连接地址
    master = connect_to_vehicle(connection_string)
    
    try:
        arm_vehicle(master)
        time.sleep(1)  # 等待2秒以确保飞控稳定
        
        # 设置PWM为前进模式

        set_rc_channel_pwm(test_channel,forward_pwm)
        print("motor forward!!")
        time.sleep(2)  # 保持前进模式5秒
        
        # 设置PWM为停止模式
        set_rc_channel_pwm(test_channel,stop_pwm)
        print("motor stop!!")
        time.sleep(2)  # 保持停止模式5秒
        disarm_vehicle(master)
    

    except KeyboardInterrupt:
        disarm_vehicle(master)
        print("Vehicle disarmed")