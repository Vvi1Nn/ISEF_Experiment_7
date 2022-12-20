// Publisher of static coordinate transformation:
//     Publishes position information about the laser coordinate system 
// Implementation steps:
//     1.Import module
//     2.Initialize ROS node
//     3.Create static coordinate broadcaster
//     4.Create and organize the broadcast message
//     5.The broadcaster sends a message
//     6.spinOnce()

#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "static_broadcast");

    tf2_ros::StaticTransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped ts;

    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link";

    ts.child_frame_id = "laser";

    ts.transform.translation.x = 0.2;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.5;

    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    broadcaster.sendTransform(ts);
    ros::spin();
    return 0;
}