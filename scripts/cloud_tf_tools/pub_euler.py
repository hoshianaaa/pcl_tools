import rospy
from std_msgs.msg import Float32MultiArray
import math

def talker():
    pub = rospy.Publisher('/cloud_tf/euler', Float32MultiArray, queue_size=10)
    r = rospy.Rate(10)

    m=[]

    for p in range(6):
        m.append(0)
    
    m[0] = math.pi / 2
    m[1] = 0
    m[2] = 0
    m[3] = 0.3
    m[4] = 0
    m[5] = 0

    while not rospy.is_shutdown():
      _forPublish = Float32MultiArray(data=m)
      pub.publish(_forPublish)
      r.sleep()
      print("pub")

rospy.init_node("_pub")
talker()

