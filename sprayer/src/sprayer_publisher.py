import rospy
import time
from std_msgs.msg import String

def led_control_publisher():
    pub = rospy.Publisher('/led_control', String, queue_size=10)
    rospy.init_node('led_control_publisher', anonymous=True)
    rate = rospy.Rate(1)  # Publish messages at 1 Hz

    while not rospy.is_shutdown():
        message = ["LED_OFF", "LED_ON"]
        while True:
            for m in message:
               pub.publish(m)  # Send LED_ON message
               rospy.loginfo(f"Published: {m}")
               time.sleep(3)
        # rospy.sleep(2)  # Keep the LED ON for 2 seconds

        # pub.publish("LED_OFF")  # Send LED_OFF message
        # rospy.loginfo("Published: LED_OFF")
        # rospy.sleep(2)  # Keep the LED OFF for 2 seconds

if __name__ == '__main__':
    try:
        led_control_publisher()
    except rospy.ROSInterruptException:
        pass
