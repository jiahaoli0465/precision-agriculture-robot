#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import serial
import time

class ArduinoController:
    def __init__(self):
        # Set up the serial connection to the Arduino
        self.arduino = serial.Serial('/dev/arduino-uno', 9600, timeout=1)  # Replace with your port
        time.sleep(2)  # Allow time for the connection to initialize

        # Initialize the ROS subscriber
        rospy.init_node('relay_control_subscriber', anonymous=True)
        rospy.Subscriber('/relay_control', String, self.callback)
        rospy.loginfo("Subscriber initialized and listening to /relay_control")

    def callback(self, msg):
        # Send the received command to the Arduino
        command = msg.data.strip()
        rospy.loginfo(f"Received: {command}")
        if command in ["RELAY_ON", "RELAY_OFF"]:
            self.arduino.write(f"{command}\n".encode())
            rospy.loginfo(f"Sent to Arduino: {command}")
        else:
            rospy.logwarn(f"Unknown command received: {command}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ArduinoController()
        controller.run()
    except rospy.ROSInterruptException:
        pass