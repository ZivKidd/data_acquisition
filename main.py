import rospy
# import PySpin
# rospy.init_node('pointgrey')
# print(rospy.Time.now())
from std_msgs.msg import String
def talker():
    pub=rospy.Publisher('ins_data',String,queue_size=10)
    rospy.init_node('ins_node',anonymous=True)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str=str(rospy.Time.now())
        print(hello_str)
        pub.publish(hello_str)
        rate.sleep()
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass
