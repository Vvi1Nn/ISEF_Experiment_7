#! /usr/bin/env python3

"""  
    Dynamic frame relative attitude Posting
    (the relative attitude of one frame with respect to another is constantly changing)

    Requirements: Launch turtlesim_node, where the form has one world coordinate system
    (the origin of the coordinate system is in the lower left corner),
    the turtle is another coordinate system, and the keyboard to control the turtle motion,
    the relative position of the two coordinate systems is dynamically published.

    Implementation analysis:
        1.The turtle can be regarded not only as a coordinate system,
          but also as a coordinate point in the world coordinate system.
        2.Subscribe to turtle1/pose to get the turtle's x and y coordinates, offset,
          and linear and angular velocities in world coordinates.
        3.Transform the pose information into coordinate system relative information and release.
    Implementation process:
        1.Import module
        2.Initialize the ROS node
        3.Subscribe /turtle1/pose topic message
        4.Callback function
            4-1.Create a TF broadcaster
            4-2.Create broadcast data (with pose Settings)
            4-3.The broadcaster publishes the data
        5.spin
"""

# 1.Import module
import rospy
import tf2_ros
import tf
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped

# 4.Callback function
def doPose(pose):
    # 4-1.Create a TF broadcaster
    broadcaster = tf2_ros.TransformBroadcaster()
    # 4-2.Create broadcast data (with pose Settings)
    tfs = TransformStamped()
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    tfs.child_frame_id = "turtle1"
    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0.0
    qtn = tf.transformations.quaternion_from_euler(0,0,pose.theta)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    # 4-3.The broadcaster publishes the data
    broadcaster.sendTransform(tfs)

if __name__ == "__main__":
    # 2.Initialize the ROS node
    rospy.init_node("dynamic_tf_pub_p")
    # 3.Subscribe /turtle1/pose topic message
    sub = rospy.Subscriber("/turtle1/pose",Pose,doPose)
    # 4.Callback function
    # 5.spin
    rospy.spin()