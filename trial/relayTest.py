import rospy
import time
from std_msgs.msg import String
rospy.init_node('test')
pub = rospy.Publisher('relay',String,queue_size=10)

for i in range(100):
    pub.publish('on');

    time.sleep(2)

    pub.publish('ff');
    time.sleep(2)


