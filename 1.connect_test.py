from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:192.168.2.1:{}'.format(14550))#  udp 是IP地址     port 是端口号
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (
            master.target_system, master.target_system))

while True:
	try:
		msg = master.recv_match(type='ATTITUDE', blocking=True)

		if not msg:
			raise ValueError()
		print(msg.to_dict())
	except KeyboardInterrupt:
		print('Key bordInterrupt! exit')
		break
