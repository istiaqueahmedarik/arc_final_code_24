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
# from detect_aruco.msg import Num
# from aruco_go.msg import wheel
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import math
rospy.init_node('mission_upload_start')
pub = rospy.Publisher('/drop', String, queue_size=10)
color = rospy.Publisher('/color',String,queue_size=1)

pub1 = rospy.Publisher('joystick', String, queue_size=10)
pub2 = rospy.Publisher('wifi', String, queue_size=10)


lidar_dist = {}
    
def obstacleAvoidance():
    try:
        rospy.loginfo("obstacle avoid mode")
        neutral_speed = 1500
        forward_speed = 1750  
        backward_speed = 1250  
        front_distance = lidar_dist[0]
        # left_distance =  lidar_dist[-90]
        left_corner_distance = lidar_dist[-180+45]
        
        right_distance = lidar_dist[180-90]
        right_corner_distance = lidar_dist[180-45]

        

        left_distance=10
        if(lidar_dist[-89]==float('inf')):
            left_distance = lidar_dist[-89]
        right_distance=1000
        for i in range(-80,-90,-1):
            if(lidar_dist[i]==float('inf')):
                continue
            
            left_distance = min(left_distance,lidar_dist[i])
            print(lidar_dist[-1*i])

            if(lidar_dist[-1*i]==float('inf')):
                continue
            right_distance = min(right_distance,lidar_dist[-1*i])
        if(left_distance == float('inf')):
            left_distance = 0.1
        if(right_distance == float('inf')):
            right_distance = 0.1
        
        left_corner_distance = 10000
        for i in range(140,179,1):
            if(lidar_dist[-1*i]==float('inf')):
                continue
            else:
                left_corner_distance = min(left_corner_distance,lidar_dist[-1*i])
        right_corner_distance = 10000
        for i in range(140,179,1):
            # right_corner_distance = min(right_corner_distance,lidar_dist[-1*i])
            if(lidar_dist[i]==float('inf')):
               continue
            else:
                right_corner_distance = min(right_corner_distance,lidar_dist[i])

        print(left_distance,right_distance,left_corner_distance,right_corner_distance)
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
            speed_diff = int((right_distance - left_distance) * 50)  
            l_speed = forward_speed
            r_speed = max(neutral_speed, forward_speed - speed_diff)
            rospy.loginfo("side check problem")


        else:
            speed_diff = int((left_distance - right_distance) * 50)  
            l_speed = max(neutral_speed, forward_speed - speed_diff)
            r_speed = forward_speed
            rospy.loginfo("here")

        # publish_custom_data((l_speed,r_speed))    
        pub1.publish("[0"+str(l_speed)+",1"+str(r_speed) +"]")   

        # pub1.publish("[0"+str(r_speed)+",1"+str(l_speed) +"]")
        # if((l_speed==1200 and r_speed==1700) or (l_speed==1700 and r_speed==1200)):
        #     pub1.publish("[01500,11500]")


    except:
        rospy.loginfo("problem")   

 



def safe():
    # disarm()
    # set_mode("MANUAL")
    pub2.publish("disconnected")
    rospy.signal_shutdown("Exiting")
    pub1.publish("[01500,11500]")
def lidar_callback(msg):
    for i, distance in enumerate(msg.ranges):
        degree = math.degrees(msg.angle_min + i * msg.angle_increment)
        lidar_dist[round(degree)] = distance
    # for i in range(-85,-90,-1):
    #         print(i,"-->",lidar_dist[i])
   
    obstacleAvoidance()
    
        


def main():

    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    pub2.publish("connected")
    color.publish('yellow')
    rospy.on_shutdown(safe)

    rospy.spin()

if __name__ == "__main__":
    main()

