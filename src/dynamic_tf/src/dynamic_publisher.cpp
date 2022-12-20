/*  
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
        1.Include header file
        2.Initialize the ROS node
        3.Create the ROS handle
        4.Create a subscription object
        5.Callback function handles subscribed data (implements TF broadcast)
            5-1.Create a TF broadcaster
            5-2.Create broadcast data (with pose Settings)
            5-3.The broadcaster publishes the data
        6.spin
*/

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

void doPose(const turtlesim::Pose::ConstPtr& pose)
{
    // 5-1.Create a TF broadcaster
    static tf2_ros::TransformBroadcaster broadcaster;
    // 5-2.Create broadcast data (with pose Settings)
    geometry_msgs::TransformStamped tfs;
    // head setting
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();

    // coordinate ID
    tfs.child_frame_id = "turtle1";

    // Coordinate system relative information setting
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0.0; // Two-dimensional implementation, no z in pose, z is 0.
    // Quaternion setting
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, pose->theta);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    // 5-3.The broadcaster publishes the data
    broadcaster.sendTransform(tfs);
}

int main(int argc, char *argv[])
{
    // 2.Initialize the ROS node
    ros::init(argc,argv,"dynamic_tf_pub");
    // 3.Create the ROS handle
    ros::NodeHandle nh;
    // 4.Create a subscription object
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, doPose);
    // 5.Callback function handles subscribed data (implements TF broadcast)
      
    // 6.spin
    ros::spin();
    return 0;
}