#! /usr/bin/env python3

"""  
    Subscribe to coordinate system information to generate a coordinate point data relative to the child coordinate system:
        Convert to a coordinate point in the parent coordinate system
    Implementation steps:
        1.Import module
        2.Initialize ROS node
        3.Create a TF subscription object
        4.Create a radar coordinate system
        5.The API for the survey subscription object converts the point coordinates in 4 to coordinates relative to the world
        6.spinOnce()
"""

import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped

if __name__ == "__main__":

    rospy.init_node("static_sub_tf_p")

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():    

        point_source = PointStamped()
        point_source.header.frame_id = "radar"
        point_source.header.stamp = rospy.Time.now()
        point_source.point.x = 10
        point_source.point.y = 2
        point_source.point.z = 3

        try:
            point_target = buffer.transform(point_source, "world")
            rospy.loginfo("Result: x = %.2f, y = %.2f, z = %.2f",
                            point_target.point.x,
                            point_target.point.y,
                            point_target.point.z)
        except Exception as e:
            rospy.logerr("Error: %s", e)

        rate.sleep()