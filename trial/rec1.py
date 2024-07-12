#!/usr/bin/env python

import rospy
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointSetCurrent
from mavros_msgs.msg import WaypointReached
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import struct
import crcmod
import serial
import time
from mavros_msgs.msg import RCOut
rospy.init_node('mission_upload_start')
pub = rospy.Publisher('/drop', String, queue_size=10)
crc16 = crcmod.mkCrcFun(0x11021, initCrc=0xFFFF, rev=False, xorOut=0x0000)

latitude = 0
longitude = 0
port = '/dev/ttyUSB0'  # Replace with your serial port
baud_rate = 9600  # Replace with your baud rate
stage_2_lat = 0
stage_2_lon = 0
ar_move = False

# Open serial port
ser = serial.Serial(port, baud_rate)
def gps_callback(data):
    global latitude, longitude
    latitude = data.latitude
    longitude = data.longitude

def encode_message(msg_id, body):
    start_byte = 0x7E
    body_length = len(body)
    frame = struct.pack('>BBB', start_byte, msg_id, body_length) + body
    checksum = crc16(frame)
    frame += struct.pack('>H', checksum)
    return frame

def send_message(serial_port, msg_id, body):
    message = encode_message(msg_id, body)
    serial_port.write(message)
    print(f"Sent Message: {message.hex()}")

def decode_message(frame):
    start_byte, msg_id, body_length = struct.unpack('>BBB', frame[:3])
    body = frame[3:-2]
    received_checksum = struct.unpack('>H', frame[-2:])[0]
    calculated_checksum = crc16(frame[:-2])
    
    if received_checksum != calculated_checksum:
        raise ValueError("Checksum does not match")

    return msg_id, body

def read_message(serial_port):
    while True:
        if serial_port.read(1) == b'\x7E':
            msg_id = serial_port.read(1)
            body_length = serial_port.read(1)
            body = serial_port.read(ord(body_length))
            checksum = serial_port.read(2)
            frame = b'\x7E' + msg_id + body_length + body + checksum
            return decode_message(frame)
pub1 = rospy.Publisher('joystick', String, queue_size=10)
pub2 = rospy.Publisher('wifi', String, queue_size=10)

def create_waypoint(lat, lon, alt):
    wp = Waypoint()
    wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp.command = 16  # MAV_CMD_NAV_WAYPOINT
    wp.is_current = False
    wp.autocontinue = True
    wp.param1 = 0  # Hold time in seconds
    wp.param2 = 0  # Acceptance radius in meters
    wp.param3 = 0  # 0 to pass through WP
    wp.param4 = float('nan')  # Desired yaw angle at waypoint
    wp.x_lat = lat
    wp.y_long = lon
    wp.z_alt = alt
    return wp

def upload_mission(waypoints):
    rospy.wait_for_service('/mavros/mission/push')
    try:
        waypoint_push = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        resp = waypoint_push(start_index=0, waypoints=waypoints)
        rospy.loginfo(f"Waypoints uploaded: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def clear_mission():
    rospy.wait_for_service('/mavros/mission/clear')
    try:
        waypoint_clear = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        resp = waypoint_clear()
        rospy.loginfo(f"Waypoints cleared: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def set_current_waypoint(seq):
    rospy.wait_for_service('/mavros/mission/set_current')
    try:
        waypoint_set_current = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)
        resp = waypoint_set_current(wp_seq=seq)
        rospy.loginfo(f"Current waypoint set: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def set_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        resp = set_mode_srv(custom_mode=mode)
        rospy.loginfo(f"Mode set to {mode}: {resp.mode_sent}")
        return resp.mode_sent
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def disarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        resp = arming_srv(value=False)
        rospy.loginfo(f"Disarmed: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def arm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        resp = arming_srv(value=True)
        rospy.loginfo(f"Armed: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def gps_callback(data):
    global latitude, longitude
    latitude = data.latitude
    longitude = data.longitude

t0 = time.time()

def waypoint_reached_callback(data):
    global last_waypoint, t0
    if data.wp_seq == last_waypoint.seq:
        rospy.loginfo("Last waypoint reached")
        disarm()
        t1 = time.time()
        taken_time = t1 - t0
        rospy.loginfo(f"Time taken: {taken_time}")
        message = f"Time taken: {taken_time}"
        send_message(ser, 0x05, message.encode('utf-8'))
        if(current_stage==1):
            pub.publish("drop")
        t0 = time.time()
def waypoint_reached_by_ar():
    global last_waypoint, t0
    rospy.loginfo("Last waypoint reached")
    disarm()
    t1 = time.time()
    taken_time = t1 - t0
    rospy.loginfo(f"Time taken: {taken_time}")
    message = f"Time taken: {taken_time}"
    send_message(ser, 0x05, message.encode('utf-8'))
    
    t0 = time.time()




last_waypoint = None

yaml_dict = ""
current_stage = 0
arm_status = False

def rc_out_callback(data):
    if(ar_move == True):
        return
    ch1 = data.channels[0]
    ch2 = data.channels[1]

    st = "0" + str(int(ch1)) + ",1" + str(int(ch2))
    pub1.publish(st)
def stage_control(stage_id):
    pass

def safe():
    disarm()
    set_mode("MANUAL")
    pub2.publish("disconnected")
    rospy.signal_shutdown("Exiting")

def main():
    global latitude, longitude, yaml_dict, current_stage, arm_status, last_waypoint, stage_2_lat, stage_2_lon

    rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_callback)
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, waypoint_reached_callback)
    rospy.Subscriber("/mavros/rc/out", RCOut, rc_out_callback)
    pub2.publish("connected")

   

    try:
        msg_id, body = read_message(ser)
        if msg_id == 0x02:  # NavigateToGPS message
            latitude1, longitude1 = struct.unpack('>dd', body)
            if(current_stage==2):
                stage_2_lat = latitude1
                stage_2_lon = longitude1
            waypoints = []
            waypoints.append(create_waypoint(latitude, longitude, 10))
            waypoints.append(create_waypoint(latitude1, longitude1, 10))
            last_waypoint = create_waypoint(latitude1, longitude1, 10)

            disarm()

            if not clear_mission():
                return

            if not upload_mission(waypoints):
                return

            if not set_current_waypoint(0):
                return

            if not set_mode("GUIDED"):
                return

            if not set_mode("AUTO"):
                return

            if not arm():
                return

            rospy.loginfo("Mission started successfully")

            send_message(ser, 0x00, b'')
        elif msg_id == 0x0A:
            yaml_dict = body.decode('utf-8')
            print(f"Received YAML serialized dict: {yaml_dict}")
            send_message(ser, 0x00, b'')
        elif msg_id == 0x04:
            stage_id = struct.unpack('>B', body)[0]
            print(f"Received Stage ID: {stage_id}")
            current_stage = stage_id
            while(latitude==0 and longitude==0):
                rospy.sleep(1)
            send_message(ser, 0x00, b'')
        elif msg_id == 0x01:
            armed = struct.unpack('>B', body)[0]
            arm_status = bool(armed)
            print(f"Received Arm/Disarm command: {armed}")
            send_message(ser, 0x00, b'')
        else:
            print("Unexpected message received")
    finally:
        ser.close()

    rospy.spin()

if __name__ == "__main__":
    main()
