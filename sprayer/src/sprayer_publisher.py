import rospy
import time
import math
from std_msgs.msg import String
import cv2
import cv_bridge
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from detector import Detector
from camera_control import CameraControl
import time
import tf2_ros


SPEED = 0.08
# gap between the robot and a plant
GAP = 0.7
RATE = 10


class Sprayer:
    def __init__(self):
        rospy.init_node('sprayer_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.relay_pub = rospy.Publisher('/relay_control', String, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.dist = None
        self.yaw = None

        self.right_dist = float('inf')
        self.left_dist = float('inf')
        self.front_dist = float('inf')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.detector = Detector()
        self.camera_control = CameraControl()

        # self.fiducial_ids = [100, 108]
        self.count = 0

        self.angular_rate = 2.0
        self.linear_rate = 1.2


        self.rate = rospy.Rate(RATE)  

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.dist = msg.x  
        self.yaw = msg.y  
    
    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        # Assuming right-hand wall following, get the distance to the wall
        averages = self.preprocessor(msg.ranges)  

        # print(averages)
        self.right_dist = averages[24]
        # print(self.right_dist)

        self.left_dist = averages[7]
        self.front_dist = averages[0]

        # # Calculate the distance error
        # error_d = self.right_dist - DESIRED_DISTANCE
        # current_time = rospy.Time.now().to_sec()
       
        # self.pid(error_d, current_time)
       
        # # Update previous error and time for the next iteration
        # self.prev_error_d = error_d
        # self.prev_time = current_time

    def sprayer_control(self, duration=3):
        msg = ["RELAY_OFF", "RELAY_ON"]
        self.relay_pub.publish(msg[1])  
        time.sleep(duration)
        self.relay_pub.publish(msg[0])  
    
    def normalize(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi  
    
    def face_pin(self, pin_id):
        """
        Rotates the robot so that it faces the target pin.
        """
        twist = Twist()
        print('face pin')          
        initial_yaw = self.yaw
        while not rospy.is_shutdown():
            try:
                pin_tf = self.tf_buffer.lookup_transform('base_link', f'pin_{pin_id}', rospy.Time())

                # Fiducial's position relative to the robot
                fid_x = pin_tf.transform.translation.x
                fid_y = pin_tf.transform.translation.y
                # Calculate the desired heading to face the fiducial
                desired_heading = math.atan2(fid_y, fid_x) 
                desired_heading = desired_heading if desired_heading >= 0 else 2 * math.pi + desired_heading
                print('desired', desired_heading)
                # Calculate the yaw difference between current yaw and desired heading
                yaw_diff = (desired_heading + initial_yaw) - self.yaw
                yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
                # Check if alignment is achieved within a small threshold
                if abs(yaw_diff) < 0.05: 
                    break
                # Rotate the robot based on the yaw difference
                twist.angular.z = yaw_diff * 0.5  # Scale the rotation by yaw_diff for finer control
                twist.linear.x = 0.0  # Ensure no forward movement during alignment
                self.cmd_vel_pub.publish(twist)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            self.rate.sleep()
        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def move_to_pin(self, pin_id):
        """
        Moves the robot to the target pin.
        """
        twist = Twist()
        threshold = 0.3  # Distance threshold to stop
        print('move to pin')
        initial_position = self.dist
        angular = 0
        while not rospy.is_shutdown():
            try:
                pin_tf = self.tf_buffer.lookup_transform('base_link', f'pin_{pin_id}', rospy.Time())
                fid_x = pin_tf.transform.translation.x
                fid_y = pin_tf.transform.translation.y
                linear = (fid_x - threshold) * self.linear_rate
                angular_error = math.atan2(fid_y, fid_x) 
                angular = angular_error * self.angular_rate - angular / 2.0
                print('distance to fiducial', linear)
                if linear < 0.001 and angular < 0.001:
                    break
                twist.linear.x = linear
                twist.angular.z = angular  
                self.cmd_vel_pub.publish(twist)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            self.rate.sleep()
        # Stop movement
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def process_plant(self):
        image = self.camera_control.capture_image()
        is_detected, plant_type  = self.detector.detect_plant(image)
        if is_detected:
            print('spraying...')
            time.sleep(2)
            print('spraying done')
        else:
            print('not a plant not spraying')
            time.sleep(2)
            print('spraying over')

    def get_pin_id(self):
        if self.count == 0:
            self.count += 1
            return 108
        return 100




    def locate(self):
        twist = Twist()
        RIGHT = - math.pi / 2 
        LEFT = math.pi / 2
        target_pin_ids = [109]
        while not rospy.is_shutdown():
            twist.linear.x = SPEED  
            for pin_id in target_pin_ids:
                time.sleep(2)
                self.face_pin(pin_id)
                time.sleep(2)
                self.move_to_pin(pin_id)
                time.sleep(2)
                print('complete navigation to ', pin_id)

            # self.cmd_vel_pub.publish(twist)
        twist.linear.x = 0.0

    def turn(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        twist = Twist()

        if not self.yaw: return
        target_yaw = self.yaw + target_yaw
        target_yaw = self.normalize(target_yaw)
        while not rospy.is_shutdown():

            if self.yaw:
                difference = self.normalize(target_yaw - self.yaw)
                if abs(difference) < 0.02:
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
                    rospy.loginfo("Finish turning to %d", target_yaw)
                    break
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = difference * 0.5

                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.linear.x = 0.0

    def normalize(self, angle):
        """Normalize angle to [-pi, pi]."""
        pi = math.pi
        return (angle + pi) % (2 * pi) - pi

    def preprocessor(self, all_ranges):
        '''
        return ranges as the average of groups of 12 values, 30 groups total
        0,7,14,22 would roughly be the index of front left back right, since lidar is counterclockwise
        '''
        ranges = [float('inf')]*30 
        range_num = len(all_ranges) 
        batch_range =  range_num // 30 
        ranges_index = 0
        index = -6
        sum = 0
        sum_list = []
        batch = 0
        actual = 0

        for i in range(241):
            curr = all_ranges[index]
            if curr != float('-inf') and not math.isnan(curr):
                sum += curr
                # sum_list.append(curr)
                actual += 1
            batch += 1
            index += 1
            if batch == batch_range:
                if actual != 0:
                    ranges[ranges_index] = sum/actual
                    # ranges[ranges_index] = min(sum_list)

                ranges_index += 1
                sum = 0
                batch = 0
                actual = 0

        # calculate average for the extra scanner points
        for i in range(241, range_num):
            curr = all_ranges[index]
            if curr != float('-inf') and not math.isnan(curr):
                sum += curr
                actual += 1
        if actual != 0:
            ranges[29] = sum/actual

        return ranges

    def test(self):
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":
    try:
        sprayer = Sprayer()
        sprayer.locate()
        # sprayer.sprayer_control(10) 
        # sprayer.test()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")
