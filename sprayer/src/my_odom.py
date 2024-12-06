#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('my_odom', Point, queue_size=1)
        self.old_pose = None 
        self.dist = 0.0
        self.yaw = 0.0
                
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose) 
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose):
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        if self.old_pose is not None:
            x_diff = cur_pose.position.x - self.old_pose.position.x
            y_diff = cur_pose.position.y - self.old_pose.position.y
            self.dist += math.sqrt(x_diff ** 2 + y_diff ** 2)
        self.old_pose = cur_pose

    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        orientations = [cur_orientation.x,
                cur_orientation.y,
                cur_orientation.z,
                cur_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        # From the robot's point of view, `odom` publishes values in: (i) [0, pi)
        # for the range [12, 6) o'clock counter-clockwise, but (ii) (0, -pi] for 
        # the range (12, 6] clockwise. The line below converts values in (ii) to
        # be in the range [pi, 2 * pi), so that we have values in [0, 2 * pi -1) 
        # for the entire [12, 12) o'clock counter-clockwise range.
        self.yaw = yaw

    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object we create below is not used as a geometric point,
        # but simply as a data container for `self.dist` and `self.yaw` so we can
        # publish it on `my_odom`.
        data = Point()
        data.x = self.dist
        data.y = self.yaw
        self.my_odom_pub.publish(data)
        print('publishing odom \n', data)


if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
