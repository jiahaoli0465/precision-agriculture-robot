import rospy
import time
import math
from std_msgs.msg import String
import cv2
import cv_bridge
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from request_utils import make_get_request, make_post_request

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
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)

        self.relay_pub = rospy.Publisher('/relay_control', String, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.dist = None
        self.yaw = None
        self.pose = None
        self.base_pose = None
        self.right_dist = float('inf')
        self.left_dist = float('inf')
        self.front_dist = float('inf')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.detector = Detector()
        self.camera_control = CameraControl()

        # self.fiducial_ids = [100, 108]

        self.fiducial_plant_image_map = {} #id: [plant_type, image]
        self.spray_time = {'Cactus': 1.5, 'Thyme':2, 'Basil': 2, 'Parsley': 3, 'Gatorade': 5}
        self.count = 0

        self.angular_rate = 2.0
        self.linear_rate = 1.2


        self.rate = rospy.Rate(RATE)  
    
    def odom_cb(self, msg):
        cur_pose = msg.pose.pose
        self.pose = cur_pose.position
        if not self.base_pose:
            self.base_pose = cur_pose.position


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

    def spray_water(self, duration=3):
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

    def process_plant(self, pin_id):
        image = self.camera_control.capture_image()
        is_detected, plant_type  = self.detector.detect_plant(image)
        if is_detected or plant_type == 'Gatorade':
            self.fiducial_plant_image_map[pin_id] = [plant_type, image]
            # self.turn(0.1)
            # print('spraying...')
            # time.sleep(2)
            # self.spray_water()
            # print('spraying done')
        else:
            print('not a plant try again')
            time.sleep(1)
            image = self.camera_control.capture_image()
            is_detected, plant_type  = self.detector.detect_plant(image)
            if is_detected or plant_type == 'Gatorade':
                self.fiducial_plant_image_map[pin_id] = [plant_type, image]

    def get_pin_id(self):
        if self.count == 0:
            self.count += 1
            return 108
        return 100


    def scan_for_fids(self):
        """
        Scans for fiducials by rotating in place. Note that the `mapper` node
        does the actual mapping.
        """
        print('scanning')
        twist = Twist()
        twist.angular.z = 0.4  # Set a low angular velocity for scanning
        scan_duration = rospy.Duration(20)  # Scan for 10 seconds
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < scan_duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        # Stop scanning
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def locate(self):
        twist = Twist()
        RIGHT = - math.pi / 2 
        LEFT = math.pi / 2
        target_pin_ids = [106]

        self.scan_for_fids()

        while not rospy.is_shutdown():
            twist.linear.x = SPEED  

            start_pose = self.pose
            
            for pin_id in target_pin_ids:
                time.sleep(2)
                self.face_pin(pin_id)
                time.sleep(2)
                self.move_to_pin(pin_id)
                time.sleep(2)
                

                # self.spray_water()
                self.process_plant(pin_id)

                time.sleep(2)

                

                





                print('complete navigation to ', pin_id)
                self.back_to_base()

            break
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

    def back_to_base(self):
        pass

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


# if __name__ == "__main__":
#     try:
#         sprayer = Sprayer()
#         sprayer.locate()
#         # sprayer.spray_water(30) 
#         # sprayer.test()
#         sprayer.relay_pub.publish("RELAY_OFF")  

#     except rospy.ROSInterruptException:
#         sprayer.relay_pub.publish("RELAY_OFF")  
#         rospy.loginfo("Shutting down")
# BLOW AND SUCK ON THAT THING if get stuck  
url = 'http://172.20.111.126:6969/'
if __name__ == "__main__":
    try:
        sprayer = Sprayer()
        # sprayer.locate()
        PREV_INSTRUCTION = 'NONE'
        while not rospy.is_shutdown():
            try:
                # Make request to Flask server
                # TODO: Since internet unstable all these post/get needs retries

                response = requests.get(f'{url}/get_instruction', timeout=1.0)
                if response.status_code == 200:
                    instruction = response.json().get('instruction', 'NONE')
                    if instruction != PREV_INSTRUCTION:

                        if instruction == 'GO_HOME':
                            rospy.loginfo("Received GO_HOME instruction")
                            # Add home movement logic here
                            sprayer.back_to_base()

                            # After completing, reset instruction
                            requests.post(f'{url}/update_instruction', 
                                        json={'instruction': 'NONE'})
                        
                        elif instruction == 'SCAN_ALL':
                            rospy.loginfo("Received SCAN_ALL instruction")
                            # turn 360 scanning for fiducials
                            sprayer.locate()

                            fiducial_plant_image_map = sprayer.fiducial_plant_image_map
                            
                            plant_list = []
                            for fid in fiducial_plant_image_map:
                                plant_list.append([
                                    fid,  # fiducial id
                                    fiducial_plant_image_map[fid][0],  # plant type
                                    fiducial_plant_image_map[fid][1]   # base64 image
                                ])

                            requests.post(f'{url}/update_plants', 
                                        json={'available_plants': plant_list})

                            requests.post(f'{url}/update_instruction', 
                                        json={'instruction': 'NONE'})

                        elif instruction.startswith('GO_TO_PLANT_'):
                            fiducial_id = instruction.split('_')[-1]
                            # if any(fiducial_id == plant[0] for plant in available_plants):

                            rospy.loginfo("Received GO_TO_PLANT instruction")

                            sprayer.face_pin(fiducial_id)
                            time.sleep(1)
                            sprayer.move_to_pin(fiducial_id)
                            sprayer.turn(0.1)
                            spray_time = sprayer.spray_time[sprayer.fiducial_plant_image_map[fiducial_id]]
                            sprayer.spray_water(spray_time)

                            sprayer.back_to_base()




                    

                            requests.post(f'{url}/update_instruction', 
                                        json={'instruction': 'NONE'})


            except requests.exceptions.RequestException as e:
                rospy.logwarn(f"Failed to fetch instructions: {e}")
            
            # Add small delay to prevent flooding
            rospy.sleep(0.5)

        sprayer.relay_pub.publish("RELAY_OFF")  

    except rospy.ROSInterruptException:
        sprayer.relay_pub.publish("RELAY_OFF")  
        rospy.loginfo("Shutting down")