import math
import rospy
from sensor_msgs.msg import LaserScan

rospy.init_node('test')
lidar_dist = {}

def lidar_callback(msg):
    for i, distance in enumerate(msg.ranges):
        degree = math.degrees(msg.angle_min + i * msg.angle_increment)
        lidar_dist[round(degree)] = distance
    for i in range(-85,-90,-1):
        print(i,"-->",lidar_dist[i])
    # for i in range(90):
        # print(180-i,"-->",lidar_dist[180i]);
        

def main():
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    rospy.spin();



if __name__ == "__main__":
    main()
