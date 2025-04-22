from pymavlink import mavutil
import time

def connect_to_vehicle(connection_string):
    # Connect to the vehicle
    print(f"Connecting to vehicle on {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Heartbeat received. Connected to vehicle.\n\t")
    return master

def arm_vehicle(master):
    # Arm the vehicle
    print("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle is armed.")

def disarm_vehicle(master):
    # Disarm the vehicle
    print("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Vehicle is disarmed.")

def monitor_status(master):
    # Monitor and print vehicle status
    print("Monitoring vehicle status...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            armed_status = "armed" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "disarmed"
            print(f"Vehicle is currently {armed_status}.")
        time.sleep(1)


def beep(master, count, duration):
    """
    Control the buzzer to beep a specified number of times.
    :param master: MAVLink connection object
    :param count: Number of beeps
    :param duration: Duration of each beep in seconds
    """
    for _ in range(count):
        # Send a MAVLink command to activate the buzzer
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY,  # Command to control relay (buzzer)
            0,
            1, 0, 0, 0, 0, 0, 0  # Turn on the relay
        )
        time.sleep(duration)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY,  # Command to control relay (buzzer)
            0,
            0, 0, 0, 0, 0, 0, 0  # Turn off the relay
        )
        time.sleep(duration)

if __name__ == "__main__":
    # Replace with your connection string, e.g., 'udp:0.0.0.0:14550'
    connection_string = 'udp:192.168.2.1:14550'
    master = connect_to_vehicle(connection_string)

    try:
        # Arm the vehicle
        arm_vehicle(master)
        time.sleep(5)  # Wait for 5 seconds

        # Disarm the vehicle
        disarm_vehicle(master)
        time.sleep(5)  # Wait for 5 seconds

        # Monitor status (press Ctrl+C to exit)
        monitor_status(master)
    except KeyboardInterrupt:
        print("Exiting...")