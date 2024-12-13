import rospy
import time
import math
import cv2
import cv_bridge
import time
import tf2_ros
import sys
import requests
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from request_utils import make_get_request, make_post_request
from detector import Detector
from camera_control import CameraControl


SPEED = 0.2
# gap between the robot and a plant
GAP = 0.7
RATE = 10
MIN_TURN_SPEED = 0.1
MAX_TURN_SPEED = 0.2
THRESHOLD = 0.5
BASE = 104
FIDUCIAL_IDS = [104, 106, 108]


class Sprayer:
    def __init__(self):
        rospy.init_node('sprayer_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.relay_pub = rospy.Publisher('/relay_control', String, queue_size=10)
        self.dist = None
        self.yaw = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.detector = Detector()
        self.camera_control = CameraControl()

        self.fiducial_plant_image_map = {} #id: [plant_type, image]
        self.spray_time = {'Cactus': 1, 'Grass':2, 'Succulent': 2, 'Parsley': 3, 'Bottle': 0}
        self.count = 0

        self.rate = rospy.Rate(RATE)  


    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.dist = msg.x  
        self.yaw = msg.y  

    
    def get_plant_type_by_id(self, fid_id):
        if self.fiducial_plant_image_map:
            if fid_id in self.fiducial_plant_image_map:
                arr = fiducial_plant_image_map[fid_id]
                plant_type = arr[0]
                return plant_type
        return 'Bottle'
    
    def get_spray_time_by_type(self, plant_type):
        if self.spray_time:
            if plant_type in self.spray_time:
                return self.spray_time[plant_type]
        return 3


    def spray_water(self, duration=3):
        msg = ["RELAY_OFF", "RELAY_ON"]
        self.relay_pub.publish(msg[1])  
        time.sleep(duration)
        self.relay_pub.publish(msg[0])  
    
    def normalize(self, angle):
        """Normalize angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def face_pin(self, pin_id):
        """
        Rotates the robot to face the target pin directly.
        """
        rospy.loginfo(f"Facing pin {pin_id}")
        initial_yaw = self.yaw  # Store the initial yaw of the robot
        twist = Twist()

        while not rospy.is_shutdown():
            try:
                pin_tf = self.tf_buffer.lookup_transform('base_link', f'pin_{pin_id}', rospy.Time())
                dx = pin_tf.transform.translation.x
                dy = pin_tf.transform.translation.y
                target_heading = math.atan2(dy, dx)
                target_heading = target_heading if target_heading >= 0 else 2 * math.pi + target_heading  # Normalize to [0, 2pi]
                yaw_diff = (target_heading + initial_yaw) - self.yaw
                yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
                if abs(yaw_diff) < 0.05:
                    break  # If aligned, exit the loop
                twist.angular.z = yaw_diff * 0.5  # Adjust angular velocity based on the yaw difference
                twist.linear.x = 0.0  # No linear movement while rotating
                self.cmd_vel_pub.publish(twist)
                break
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                rospy.logwarn(f"Could not face pin {pin_id}")
                self.rate.sleep()

    def move_to_pin(self, pin_id):
        """
        Moves the robot to the target pin while avoiding overshooting.
        """
        rospy.loginfo(f"Moving to pin {pin_id}")
        twist = Twist()

        while not rospy.is_shutdown():
            try:
                pin_tf = self.tf_buffer.lookup_transform('base_link', f'pin_{pin_id}', rospy.Time())
                dx = pin_tf.transform.translation.x
                dy = pin_tf.transform.translation.y
                distance = math.hypot(dx, dy)

                if distance < THRESHOLD:# If within threshold, stop
                    rospy.loginfo(f"Reached pin {pin_id}")
                    break

                linear_speed = min(distance * 0.5, SPEED)
                angular_error = math.atan2(dy, dx)
                angular_speed = angular_error * 0.9

                twist.linear.x = linear_speed
                twist.angular.z = angular_speed
                self.cmd_vel_pub.publish(twist)
                self.rate.sleep()

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                rospy.logwarn(f"Could not move to pin {pin_id}")
                self.rate.sleep()

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def process_plant(self, pin_id):
        image = self.camera_control.capture_image()
        is_detected, plant_type  = self.detector.detect_plant(image)
        if is_detected or plant_type == 'Bottle':
            self.fiducial_plant_image_map[pin_id] = [plant_type, image]
        else:
            print('not a plant try again')
            time.sleep(1)
            image = self.camera_control.capture_image()
            is_detected, plant_type  = self.detector.detect_plant(image)
            if is_detected or plant_type == 'Bottle':
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

        self.scan_for_fids()

        while not rospy.is_shutdown():
            twist.linear.x = SPEED  
            
            for pin_id in FIDUCIAL_IDS:
                if pin_id == BASE:
                    continue
                self.face_pin(pin_id)
                self.move_to_pin(pin_id)
                self.process_plant(pin_id)

                print('complete navigation to ', pin_id)

            break
        self.face_pin(BASE)
        self.move_to_pin(BASE)

            # self.cmd_vel_pub.publish(twist)
        twist.linear.x = 0.0



url = 'http://172.20.111.126:6969/'
# BLOW AND SUCK ON THAT THING if get stuck  
if __name__ == "__main__":

    try: 
        test = sys.argv[1]
        if test == 'test': test = True
    except:
        test = False

    sprayer = Sprayer()
    
    if not test:
        try:
            PREV_INSTRUCTION = 'NONE'
            
            while not rospy.is_shutdown():
                try:
                    # Make request to Flask server with retry logic
                    response = make_get_request(f'{url}/get_instruction', timeout=1.0)
                    
                    if response and response.status_code == 200:  # Check if response exists first
                        instruction = response.json().get('instruction', 'NONE')
                        
                        if instruction != PREV_INSTRUCTION:
                            rospy.loginfo(f"New instruction received: {instruction}")
                            PREV_INSTRUCTION = instruction  # Update PREV_INSTRUCTION when processing new one
                            
                            if instruction == 'GO_HOME':
                                rospy.loginfo("Executing GO_HOME instruction")
                                sprayer.move_to_pin(BASE)
                                
                                # Update instruction with retry logic
                                make_post_request(f'{url}/update_instruction', 
                                            {'instruction': 'NONE'})
                            
                            elif instruction == 'SCAN_ALL':
                                rospy.loginfo("Executing SCAN_ALL instruction")
                                sprayer.locate()

                                fiducial_plant_image_map = sprayer.fiducial_plant_image_map
                                plant_list = [
                                    [fid, fiducial_plant_image_map[fid][0], fiducial_plant_image_map[fid][1]]
                                    for fid in fiducial_plant_image_map
                                ]

                                # Update plants with retry logic
                                make_post_request(f'{url}/update_plants', 
                                            {'available_plants': plant_list})
                                
                                make_post_request(f'{url}/update_instruction', 
                                            {'instruction': 'NONE'})

                            elif instruction.startswith('GO_TO_PLANT_'):
                                fiducial_id = instruction.split('_')[-1]
                                rospy.loginfo(f"Executing GO_TO_PLANT instruction for ID: {fiducial_id}")
                                fidcuial_id = int(fiducial_id)
                                try:
                                    sprayer.face_pin(fiducial_id)
                                    time.sleep(1)
                                    sprayer.move_to_pin(fiducial_id)
                                    print('sprayer_time', sprayer.spray_time)
                                    print('fiducial_id', fidcuial_id)

                                    plant_type = sprayer.get_plant_type_by_id(fidcuial_id)
  
                                    print('plant_type ', plant_type)

                                    spray_time = sprayer.get_spray_time_by_type(plant_type)
                                    # sprayer.turn(0.05)
                                    sprayer.spray_water(spray_time)
                                    sprayer.face_pin(BASE)
                                    sprayer.move_to_pin(BASE)

                                    make_post_request(f'{url}/update_instruction', 
                                                {'instruction': 'NONE'})
                                except Exception as e:
                                    rospy.logerr(f"Error during plant operation: {e}")
                                    # Still try to reset instruction on failure
                                    make_post_request(f'{url}/update_instruction', 
                                                {'instruction': 'NONE'})

                except Exception as e:
                    rospy.logwarn(f"Unexpected error: {e}")
                
                rospy.sleep(0.5)

            sprayer.relay_pub.publish("RELAY_OFF")

        except rospy.ROSInterruptException:
            sprayer.relay_pub.publish("RELAY_OFF")
            rospy.loginfo("Shutting down")

    else:
        try:

            sprayer.face_pin(BASE)
            sprayer.move_to_pin(BASE)

        except rospy.ROSInterruptException:
            pass








