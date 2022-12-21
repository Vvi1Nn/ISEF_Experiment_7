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
        3.Create TF subscriber
        4.data processing
"""

# 1.Import module
import rospy
import tf2_ros
# Do not use geometry_msgs, you need to use tf2's built-in message type
from tf2_geometry_msgs import PointStamped
# from geometry_msgs.msg import PointStamped

if __name__ == "__main__":
    # 2.Initialize the ROS node
    rospy.init_node("static_sub_tf_p")
    # 3.Create TF subscriber
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():    
        # 4.Create a radar coordinate system
        point_source = PointStamped()
        point_source.header.frame_id = "turtle1"
        point_source.header.stamp = rospy.Time.now()
        point_source.point.x = 10
        point_source.point.y = 2
        point_source.point.z = 3

        try:
        # 5.The API for the survey subscription object converts the point coordinates in 4 to coordinates relative to the world
            point_target = buffer.transform(point_source,"world",rospy.Duration(1))
            rospy.loginfo("Results: x = %.2f, y = %.2f, z = %.2f",
                            point_target.point.x,
                            point_target.point.y,
                            point_target.point.z)
        except Exception as e:
            rospy.logerr("Error: %s", e)

        # 6.spin
        rate.sleep()