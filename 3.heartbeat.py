from pymavlink import mavutil
import time

# 建立MAVLink连接（以UDP为例）
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

# 定义心跳包发送频率（单位：秒）
HEARTBEAT_INTERVAL = 1

def send_heartbeat():
    """发送心跳包"""
    master.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_GCS,          # 地面站类型
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID, # 地面站无自动驾驶仪
        base_mode=0,                                 # 无模式标志
        custom_mode=0,                               # 无自定义模式
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE, # 地面站状态为活跃
    )

def parse_heartbeat(msg, heart_beat_count):
    """解析并显示心跳包信息"""
    print("\n----- Heartbeat Receive %d -----"%(heart_beat_count))
    print(f"System_ID: {msg.get_srcSystem()}")        # 发送端系统ID
    print(f"Component_ID: {msg.get_srcComponent()}")     # 发送端组件ID
    print(f"Equitment_Type: {mavutil.mavlink.enums['MAV_TYPE'][msg.type].name}")  # 设备类型（如飞控、地面站）
    print(f"ArduSub_Type: {mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name}")  # 飞控型号
    print(f"System_Status: {mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name}")  # 状态（如启动中、活跃）
    print(f"Current_Mode: {mavutil.mavlink.mode_string_v10(msg)}")  # 飞行模式（如MANUAL、GUIDED）

def main():
    last_heartbeat_time = time.time()
    heart_beat_count = 0
    while True:
        # 发送心跳包（按固定频率）
        if time.time() - last_heartbeat_time > HEARTBEAT_INTERVAL:
            heart_beat_count += 1
            send_heartbeat()
            last_heartbeat_time = time.time()

        # 接收消息
        try:
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg is not None:
                parse_heartbeat(msg, heart_beat_count)
            
            # 可选：处理其他消息（如姿态、位置）
            # else_msg = master.recv_match(blocking=False)
            # if else_msg:
            #     print(f"收到其他消息: {else_msg.get_type()}")

        except Exception as e:
            print(f"Recived Error: {e}")

        time.sleep(0.1)

if __name__ == "__main__":
    main()