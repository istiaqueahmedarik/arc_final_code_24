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
from detect_aruco.msg import Num
from aruco_go.msg import wheel
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import math
rospy.init_node('mission_upload_start')
pub = rospy.Publisher('/drop', String, queue_size=10)
color = rospy.Publisher('/color',String,queue_size=1)
crc16 = crcmod.mkCrcFun(0x11021, initCrc=0xFFFF, rev=False, xorOut=0x0000)

latitude = 0
longitude = 0
port = '/dev/ttyUSB1'  # Replace with your serial port
baud_rate = 115200  # Replace with your baud rate
stage_2_lat = 0
stage_2_lon = 0
ar_move = False
tups = []
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

def distance_gps(lat1, lon1, lat2, lon2):
    R = 6371000  # radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) * math.sin(dphi / 2) + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) * math.sin(dlambda / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def gps_callback(data):
    global latitude, longitude
    latitude = data.latitude
    longitude = data.longitude
    if(stage_2_lat!=0 and stage_2_lon!=0 and current_stage==2):
        dist = distance_gps(latitude, longitude, stage_2_lat, stage_2_lon)
        if(dist<2):
            time_taken = time.time()-t0
            message = f"Time taken: {time_taken}"
            send_message(ser, 0x05, message.encode('utf-8'))
            stage_2_lat = 0
            stage_2_lon = 0

lidar_dist = {}
    
def obstacleAvoidance():
    global OBSTACLE_AVOIDANCE_FLAG
    OBSTACLE_AVOIDANCE_FLAG=True
    rospy.loginfo("obstacle avoid mode")
    try:
        rospy.loginfo("obstacle avoid mode")
        neutral_speed = 1500
        forward_speed = 1750  
        backward_speed = 1250  
        front_distance = lidar_dist[0]
        left_distance =  lidar_dist[90]
        left_corner_distance = lidar_dist[45]
        
        right_distance = lidar_dist[-90]
        right_corner_distance = lidar_dist[-45]

        

        left_distance=1000
        right_distance=1000
        for i in range(85,90):
            if(lidar_dist[i]==float('inf')):
                continue
            
            left_distance = min(left_distance,lidar_dist[i])
            if(lidar_dist[-1*i]==float('inf')):
                continue
            right_distance = min(right_distance,lidar_dist[-1*i])
        if(left_distance == float('inf')):
            left_distance = 0.1
        if(right_distance == float('inf')):
            right_distance = 0.1
        
        left_corner_distance = 10000
        for i in range(15,47):
            if(lidar_dist[-180+i]==float('inf')):
                continue
            else:
                left_corner_distance = min(left_corner_distance,lidar_dist[-180+i])
        right_corner_distance = 10000
        for i in range(15,47):
            # right_corner_distance = min(right_corner_distance,lidar_dist[-1*i])
            if(lidar_dist[180-i]==float('inf')):
               continue
            else:
                right_corner_distance = min(right_corner_distance,lidar_dist[-1*i])

        print(left_corner_distance,right_corner_distance)
        l_speed = neutral_speed
        r_speed = neutral_speed
        # if(lidar_dist[0]<1):
        #     if(left_distance>right_distance):
        #         # publish_custom_data((1800,1100))
        #         pub1.publish("[01700,11200]")
        #         time.sleep(0.5)
        #         pub1.publish("[01500,11500]")
        #         time.sleep(0.5)
        #         rospy.loginfo("left_distance>right_distance")
                
        #         return
        #     else:
        #         # publish_custom_data((1100,1800))
        #         pub1.publish("[01200,11700]")
        #         time.sleep(.5)
        #         pub1.publish("[01500,11500]")
        #         time.sleep(0.5)
        #         rospy.loginfo("left_distance>right_distance_else")


        #         return
        if  left_corner_distance < 1.3 and right_corner_distance >1.3:
            l_speed = 1700
            r_speed = 1200
            rospy.loginfo("left corner check problem")
        elif  right_corner_distance < 1 and left_corner_distance >1:
            l_speed = 1200
            r_speed = 1700
            rospy.loginfo("right corner check problem")

        elif( left_corner_distance>1.8 and right_corner_distance>1.8):
            if left_corner_distance < right_corner_distance:
                speed_diff = int((right_corner_distance - left_corner_distance) * 500)  
                l_speed = forward_speed
                r_speed = max(neutral_speed, forward_speed - speed_diff)
            else:
                speed_diff = int((left_corner_distance - right_corner_distance) * 500)  
                l_speed = max(neutral_speed, forward_speed - speed_diff)
                r_speed = forward_speed
            # l_speed = forward_speed
            # r_speed = forward_speed
            rospy.loginfo("managing corner check problem")

            
        elif left_distance < right_distance:
            speed_diff = int((right_distance - left_distance) * 500)  
            l_speed = forward_speed
            r_speed = max(neutral_speed, forward_speed - speed_diff)
            rospy.loginfo("side check problem")


        else:
            speed_diff = int((left_distance - right_distance) * 500)  
            l_speed = max(neutral_speed, forward_speed - speed_diff)
            r_speed = forward_speed
            rospy.loginfo("here")

        # publish_custom_data((l_speed,r_speed))    
        pub1.publish("[0"+str(l_speed)+",1"+str(r_speed) +"]")   
        time.sleep(0.5);

        # pub1.publish("[0"+str(r_speed)+",1"+str(l_speed) +"]")
        if((l_speed==1200 and r_speed==1700) or (l_speed==1700 and r_speed==1200)):
            time.sleep(0.5);
            pub1.publish("[01500,11500]")
            time.sleep(0.5);


    except:
        rospy.loginfo("problem")   


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
findOnce = False
OBSTACLE_AVOIDANCE_FLAG = False

def aruco_path_for_one(tuple):
    global findOnce,OBSTACLE_AVOIDANCE_FLAG
    if(OBSTACLE_AVOIDANCE_FLAG):
        return
    if(findOnce):
        # publish_custom_data((1500,1500))
        pub1.publish("[01500,11500]")
        return
    global rotate
    dis = float(tuple[2])
    x = float(tuple[1])
    id = int(tuple[0])
    if(dis<2):
        OBSTACLE_AVOIDANCE_FLAG = True
        return

    if(dis<8.0):
        
        # publish_custom_data((1200,1800));
        rospy.loginfo('here is the issue')
        pub1.publish("[01400,11600]")

        time.sleep(1)
        pub1.publish("[01500,01500]");
        time.sleep(1)
        return
    print(dis, x, id)
    if(dis<0.45):
        # return publish_custom_data((1500, 1500))
        pub1.publish("[01500,11500]")
        return
    
    else:
        left = 1500
        right = 1500

        if x > 350:
            left = 1700
            right = 1300
        elif x < -350:
            left = 1300
            right = 1700
        elif x > 50:
            right = int(1700 - map_(x))
            left = int(1500 + map_(x))
        
            
        elif x < -50:
            left = int(1700 - map_(x))
            right = int(1500 + map_(x))
            
        else:
            right = 1700
            left = 1700
        rospy.loginfo("ekta aruco er dike agaitese")
        # After processing, publish custom data
        # publish_custom_data((left, right))
        pub1.publish("[0"+str(left)+",1"+str(right)+"]")
        # time.sleep(0.5)
        if((left==1700 and right==1300) or (left==1300 and right==1700)):
            time.sleep(0.5)
            pub1.publish("[01500,11500]")

def map_(x):
    return (abs(x) * 200) / 350
def map__(x):
    return (abs(x)*200)/100
def aruco_path_for_two(tuple1, tuple2):
    global findOnce
    rospy.loginfo("2 ta aruco i paise")
    dis1 = float(tuple1[2])
    x1 = float(tuple1[1])
    id1 = int(tuple1[0])

    dis2 = float(tuple2[2])
    x2 = float(tuple2[1])
    id2 = int(tuple2[0])
    
    if(min(dis1,dis2)>=8.0):
        rospy.loginfo("8 er beshi distance so ektar dike agaitese")
        if(findOnce):
            rospy.loginfo("2 ta aruco ekbar paise so theme gese")
            # publish_custom_data((1500,1500))
            pub1.publish("[01500,11500]")
            return
        if(dis1>dis2):
            rospy.loginfo("beshi dist jetar shedike jaitese")
            aruco_path_for_one(tuple1)
        else:
            aruco_path_for_one(tuple2)

        return

    x = (x1 + x2)/2
    global OBSTACLE_AVOIDANCE_FLAG
    if(dis1<=4 or dis2<=4):
        rospy.loginfo("6 er kom distance 2 ta aruco er jekono ekta so theme jabe")
        # needs to align
        # print(x)
        OBSTACLE_AVOIDANCE_FLAG=True
        obstacleAvoidance()
        #publish_custom_data((1500,1500))
        return
    
    print(x1, x2, x,dis1,dis2)
    findOnce = True
    rospy.loginfo("2 ta aruco er majhe jaitese")
    left = 1500
    right = 1500
    if x > 350:
        left = 1700
        right = 1300
    elif x < -350:
        left = 1300
        right = 1700
    elif x > 60:
        right = int(1700 - map_(x))
        left = int(1500 + map_(x))
    
        
    elif x < -60:
        left = int(1700 - map_(x))
        right = int(1500 + map_(x))
        
    else:
        right = 1700
        left = 1700

    # After processing, publish custom data
    # publish_custom_data((left, right))
    pub1.publish("[0"+str(left)+",1"+str(right)+"]")
    if((left==1700 and right==1300) or (left==1300 and right==1700)):
            time.sleep(0.5)
            pub1.publish("[01500,11500]")

last_waypoint = None

yaml_dict = ""
current_stage = 0
arm_status = False

def rc_out_callback(data):
    if(ar_move == True):
        return
    ch1 = data.channels[0]
    ch2 = data.channels[1]

    st = "[0" + str(int(ch1)) + ",1" + str(int(ch2))+"]"
    pub1.publish(st)
def stage_control(stage_id):
    pass

def safe():
    # disarm()
    # set_mode("MANUAL")
    pub2.publish("disconnected")
    rospy.signal_shutdown("Exiting")
lastArCodeSearch = 0
def aruco_detect_callback(msg):
    global lastArCodeSearch
    if(ar_move == False):
        return
    tot = msg.total
    lst = tot.split(',')
    tuples = []

    for i in range(0, len(lst), 3):
        if int(lst[i]) == 269:
            lastArCodeSearch = time.time()

            tuples.append((float(lst[i]), float(lst[i + 1]), float(lst[i + 2])))
            
    global tups

    tups = tuples

def lidar_callback(msg):
    print(OBSTACLE_AVOIDANCE_FLAG)
    global entrance
    global lidar_dist
    for i, distance in enumerate(msg.ranges):
        degree = math.degrees(msg.angle_min + i * msg.angle_increment)
        lidar_dist[round(degree)] = distance
    if OBSTACLE_AVOIDANCE_FLAG:
        obstacleAvoidance()
        return
    if(ar_move==True):
        rospy.loginfo("check if aruco is in front")
        print(tups)
        time.sleep(1)
        if(time.time()-lastArCodeSearch>2):
            rospy.loginfo("rotating")
            pub1.publish("[01700,11300]")
            time.sleep(1)
            
            pub1.publish("[01500,11500]")
            time.sleep(1)
            pass
        else:
            if(len(tups)==2):
                rospy.loginfo("find two aruco so try to follow it")
                aruco_path_for_two(tups[0],tups[1])
                tups.clear()
            elif(len(tups)==1):
                rospy.loginfo("find one move to that direction")
                aruco_path_for_one(tups[0]);
                print(tups[0])
                tups.clear();
        
            

    
        

def main():
    global latitude, longitude, yaml_dict, current_stage, arm_status, last_waypoint, stage_2_lat, stage_2_lon

    rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_callback)
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, waypoint_reached_callback)
    rospy.Subscriber("/mavros/rc/out", RCOut, rc_out_callback)
    rospy.Subscriber("aruco_info", Num, aruco_detect_callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    pub2.publish("connected")
    color.publish('yellow')

   

    try:
        msg_id, body = read_message(ser)
        if msg_id == 0x02:  # NavigateToGPS message
            latitude1, longitude1 = struct.unpack('>dd', body)
            if(current_stage==2):
                stage_2_lat = latitude1
                stage_2_lon = longitude1
            # waypoints = []
            # waypoints.append(create_waypoint(latitude, longitude, 10))
            # waypoints.append(create_waypoint(latitude1, longitude1, 10))
            # last_waypoint = create_waypoint(latitude1, longitude1, 10)

            # disarm()

            # if not clear_mission():
            #     return

            # if not upload_mission(waypoints):
            #     return

            # if not set_current_waypoint(0):
            #     return

            # if not set_mode("GUIDED"):
            #     return

            # if not set_mode("AUTO"):
            #     return

            # if not arm():
            #     return

            rospy.loginfo("Mission started successfully")

            send_message(ser, 0x00, b'')
        elif msg_id == 0x0A:
            yaml_dict = body.decode('utf-8')
            print(f"Received YAML serialized dict: {yaml_dict}")
            send_message(ser, 0x00, b'')
            color.publish('red')
        elif msg_id == 0x04:
            stage_id = struct.unpack('>B', body)[0]
            print(f"Received Stage ID: {stage_id}")
            
            current_stage = stage_id
            while(latitude==0 and longitude==0):
                rospy.sleep(1)
            send_message(ser, 0x00, b'')
        elif msg_id == 0x06:
            tag_id, dictionary = struct.unpack('>IB', body)
            print(f"Received Aruco Tag: {tag_id}, Dictionary: {dictionary}")
            global ar_move
            ar_move = True
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
