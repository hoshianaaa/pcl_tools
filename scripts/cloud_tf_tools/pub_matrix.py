import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher('/cloud_tf/matrix', Float32MultiArray, queue_size=10)
    r = rospy.Rate(10)

    m=[]

    for p in range(12):
        m.append(0)

    l = 0
    m[l*4 + 0] = 1
    m[l*4 + 1] = 0
    m[l*4 + 2] = 0
    m[l*4 + 3] = 0.3

    l = 1
    m[l*4 + 0] = 0
    m[l*4 + 1] = 1
    m[l*4 + 2] = 0
    m[l*4 + 3] = 0

    l = 2
    m[l*4 + 0] = 0
    m[l*4 + 1] = 0
    m[l*4 + 2] = 1
    m[l*4 + 3] = 0

    while not rospy.is_shutdown():
      _forPublish = Float32MultiArray(data=m)
      pub.publish(_forPublish)
      r.sleep()
      print("pub")

rospy.init_node("_pub")
talker()

