import rospy
# import PySpin
# rospy.init_node('pointgrey')
# print(rospy.Time.now())
from std_msgs.msg import String
# from geometry_msgs.msg import
import serial

portx = "/dev/ttyUSB1"
bps = 115200
timex = 5

ser = serial.Serial(portx, bps, timeout=timex)

pub = rospy.Publisher('ins_data', String, queue_size=10)
rospy.init_node('ins_node', anonymous=True)
rate = rospy.Rate(10)

def talker():

    while not rospy.is_shutdown():

        if ser.in_waiting:
            str1=ser.readline().decode("gbk")
            str1=str(rospy.Time.now())+','+str1
            pub.publish(str1)
            if(str1=="exit"):
                break
            else:
                print(str1)

        # hello_str=str(rospy.Time.now())
        # print(hello_str)
        # pub.publish(hello_str)
        rate.sleep()
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass
