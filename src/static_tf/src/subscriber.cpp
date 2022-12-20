// Subscribe to coordinate system information to generate a coordinate point data relative to the child coordinate system:
//     Convert to a coordinate point in the parent coordinate system
// Implementation steps:
//     1.Import module
//     2.Initialize ROS node
//     3.Create a TF subscription object
//     4.Create a radar coordinate system
//     5.The API for the survey subscription object converts the point coordinates in 4 to coordinates relative to the world
//     6.spinOnce()

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "static_subscriber");

    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(0.5);
    while (ros::ok())
    {
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "laser";
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 2;
        point_laser.point.y = 3;
        point_laser.point.z = 5;

        // Use a try statement or sleep, otherwise coordinate conversion may fail due to delayed cache reception
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"base_link");
            ROS_INFO("New coordinate: (%.2f,%.2f,%.2f), Reference coordinate system is: %s", point_base.point.x, point_base.point.y, point_base.point.z, point_base.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            ROS_INFO("Error: %s", e.what());
        }

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}