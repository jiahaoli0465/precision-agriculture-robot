#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

FID_IDS = [104, 106, 108]

class Mapper:
    """
    When an instance `m` of the `Mapper` class is `run`, for every fiducial with
    id `i` detected by the robot, `m` generates a `pin_i` frame that:
    
    1. has the same origin as the `fiducial_i` frame;
    2. has the same orientation as the `odom` frame; and
    3. is a child of the `odom frame` in the tf tree.

    This assumes that the id of the said fiducial is in the array `fid_ids` given
    as an argument of `set_pins`.
    """

    def __init__(self, fid_ids=FID_IDS):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pin_dict = {}
        self.fid_ids = fid_ids

    def set_pins(self):
        """
        Build the `pin_dict` that, for every id `i` in `fid_ids`, provides a 
        `(tfs, mapped)` pair, where `tfs` is the frame of `pin_i`, and `mapped`
        is a boolean indicating whether `pin_i` has been mapped. 
        """
        for id in self.fid_ids:
            tfs = TransformStamped()
            tfs.header.frame_id = 'odom'
            tfs.child_frame_id = f'pin_{id}'
            self.pin_dict[id] = {'tfs': tfs, 'mapped': False}

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            for fid_id, pin in self.pin_dict.items():
                try:
                    self.map_pin(fid_id, pin)  
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException
                    ):
                    pass
                finally:
                    if pin['mapped']:
                        pin['tfs'].header.stamp = rospy.Time.now()
                        self.tf_broadcaster.sendTransform(pin['tfs'])
            rate.sleep()
    
    def map_pin(self, fid_id, pin):
        """
        Helper to `run`. Maps the pin; i.e., sets its translation from its parent
        `odom` frame and its rotation.
        """
        odom_to_fid_tf = self.tf_buffer.lookup_transform(
                            'odom',
                            f'fiducial_{fid_id}',
                            rospy.Time()).transform
        pin['tfs'].transform.translation = odom_to_fid_tf.translation

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        (pin['tfs'].transform.rotation.x,
        pin['tfs'].transform.rotation.y,
        pin['tfs'].transform.rotation.z,
        pin['tfs'].transform.rotation.w) = q

        pin['mapped'] = True


if __name__ == '__main__':
    rospy.init_node('mapper')
    print('running mapper')
    mapper = Mapper(FID_IDS)
    mapper.set_pins()
    mapper.run()
