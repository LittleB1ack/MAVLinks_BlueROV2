from pymavlink import mavutil
import time

# 连接飞控
master = mavutil.mavlink_connection('udp:192.168.2.1:14550')  # 端口号为 14550
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (
            master.target_system, master.target_component))

while True:
    try:
        # 接收姿态数据
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if not msg:
            raise ValueError("未接收到姿态数据")
        
        # 将姿态数据分行打印
        attitude = msg.to_dict()
        print(f"Roll: {57.2958*attitude['roll']:.2f} rad")
        print(f"Pitch: {57.2958*attitude['pitch']:.2f} rad")
        print(f"Yaw: {57.2958*attitude['yaw']:.2f} rad")
        print(f"Rollspeed: {attitude['rollspeed']:.2f} rad/s")
        print(f"Pitchspeed: {attitude['pitchspeed']:.2f} rad/s")
        print(f"Yawspeed: {attitude['yawspeed']:.2f} rad/s")
        print("-" * 30)  # 分隔线


        # 等待 1 秒
        time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
        break
    except Exception as e:
        print(f"发生错误: {e}")