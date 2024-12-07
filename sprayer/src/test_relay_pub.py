import rospy
from std_msgs.msg import String

relay_pub = rospy.Publisher('/relay_control', String, queue_size=10)
rospy.init_node('relay_node', anonymous=True)
rate = rospy.Rate(1)  
message = ["RELAY_OFF", "RELAY_ON"]
msg = message[1]
# if is_power_on:
#     msg = message[1]
relay_pub.publish(msg)  