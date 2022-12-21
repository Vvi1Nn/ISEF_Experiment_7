#!/usr/bin/env python3

"""  
    Implementation steps:   
        1.Import module
        2.Initialize the ros node
        3.Create a TF subscription object
        4.Figure out the coordinates of son1 with respect to son2
        5.Create a coordinate point dependent on son1 and call the API to figure out the coordinates of that point in son2
        6.spin
"""

# 1.Import module
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped

if __name__ == "__main__":

    # 2.Initialize the ros node
    rospy.init_node("frames_sub_p")
    # 3.Create a TF subscription object
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            # 4.Figure out the coordinates of son1 with respect to son2
            # lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            tfs = buffer.lookup_transform("son2","son1",rospy.Time(0))
            rospy.loginfo("Relative relation between son1 and son2:")
            rospy.loginfo("Parent coordinate system: %s", tfs.header.frame_id)
            rospy.loginfo("Child coordinate system: %s", tfs.child_frame_id)
            rospy.loginfo("Relative coordinates: x = %.2f, y = %.2f, z = %.2f",
                        tfs.transform.translation.x,
                        tfs.transform.translation.y,
                        tfs.transform.translation.z,)
            
            # 5.Create a coordinate point dependent on son1 and call the API to figure out the coordinates of that point in son2
            point_source = PointStamped()
            point_source.header.frame_id = "son1"
            point_source.header.stamp = rospy.Time.now()
            point_source.point.x = 1
            point_source.point.y = 1
            point_source.point.z = 1

            point_target = buffer.transform(point_source,"son2",rospy.Duration(0.5))

            rospy.loginfo("point_target Coordinate system: %s", point_target.header.frame_id)
            rospy.loginfo("The coordinates with respect to son2: (%.2f, %.2f, %.2f)",
                        point_target.point.x,
                        point_target.point.y,
                        point_target.point.z)

        except Exception as e:
            rospy.logerr("Error: %s", e)

        rate.sleep()
    # 6.spin    
    # rospy.spin()