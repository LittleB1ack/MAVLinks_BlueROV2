from pymavlink import mavutil
import time

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
    print("Vehicle is armed.")

def disarm_vehicle(master):
    """锁定飞控"""
    print("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Vehicle is disarmed.")

def set_servo_pwm(master, channel, pwm):
    """设置指定通道的PWM值（单位：微秒）"""
    print(f"Setting PWM on channel {channel} to {pwm} us")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm,
        0, 0, 0, 0, 0
    )

def get_servo_pwm(master, channel):
    """读取指定通道的PWM值"""
    param_name = f"SERVO{channel}_FUNCTION"
    print(f"Requesting PWM value for channel {channel}...")
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode(),
        -1  # 索引值为-1表示按名称请求
    )
    while True:
        message = master.recv_match(type='PARAM_VALUE', blocking=True)
        if message and message.param_id.decode('utf-8') == param_name:
            print(f"Channel {channel} PWM value: {message.param_value}")
            return message.param_value

def configure_servo_function(master, channel, function=0):
    """配置舵机通道功能为手动模式（需飞控支持）"""
    param_name = f"SERVO{channel}_FUNCTION"
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode(),
        function,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    print(f"Set {param_name} to {function}")

if __name__ == "__main__":
    connection_string = 'udp:192.168.2.1:14550'  # 修改为你的连接地址
    master = connect_to_vehicle(connection_string)

    try:
        # 解锁飞控
            # 配置 MAIN OUT 通道 1（通常为通道 1）为手动模式
            main_channel = 1  # MAIN OUT 通道 1 对应的编号
            configure_servo_function(master, main_channel, 0)
            
            # 解锁飞控
            arm_vehicle(master)
            time.sleep(1)

            # 控制 AUX OUT 通道 1 的舵机扫动
            print(f"Controlling servo on AUX OUT channel {main_channel}...")
            for pwm in range(500,2000, 100):
                set_servo_pwm(master, main_channel, pwm)
                time.sleep(0.5)  # 调整延时控制转速

            # 返回中位并锁定
            set_servo_pwm(master, main_channel, 1500)
            time.sleep(1)
            disarm_vehicle(master)
    except KeyboardInterrupt:
        disarm_vehicle(master) 
        print("Program interrupted.")
