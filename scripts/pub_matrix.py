import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher('/cloud_tf/matrix', Float32MultiArray, queue_size=10)
    r = rospy.Rate(10)

    array=[]

    for p in range(12):
        array.append(p)

    while not rospy.is_shutdown():
      array_forPublish = Float32MultiArray(data=array)
      pub.publish(array_forPublish)
      r.sleep()
      print("pub")

rospy.init_node("array_pub")
talker()

