#! /usr/bin/env python3

"""  
    Publisher of static coordinate transformation:
        Publishes position information about the laser coordinate system 
    Implementation steps:
        1.Import module
        2.Initialize ROS node
        3.Create static coordinate broadcaster
        4.Create and organize the broadcast message
        5.The broadcaster sends a message
        6.spinOnce()
"""

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":

    rospy.init_node("static_tf_pub_p")

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    tfs = TransformStamped()
    # header
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    tfs.header.seq = 101
    # child coordinate
    tfs.child_frame_id = "radar"
    # relative information of coordinate system
    tfs.transform.translation.x = 0.2
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.5

    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]

    broadcaster.sendTransform(tfs)

    rospy.spin()