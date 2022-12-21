// 1.Include header file
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // Note: Calling transform must include this header file

int main(int argc, char *argv[])
{
    // 2.Initialize the ROS node
    ros::init(argc, argv, "dynamic_tf_sub");
    ros::NodeHandle nh;
    // 3.Create a TF subscription node
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while (ros::ok())
    {
        // 4.Generate a coordinate point (relative to the child coordinate system)
        geometry_msgs::PointStamped point_turtle;
        point_turtle.header.frame_id = "turtle1";
        point_turtle.header.stamp = ros::Time();
        point_turtle.point.x = 1;
        point_turtle.point.y = 1;
        point_turtle.point.z = 0;
        // 5.Transform coordinate point (relative to parent coordinate system)
        // Create a new coordinate point to receive the result of the transformation  
        // Use a try statement or sleep, otherwise coordinate conversion may fail due to delayed cache reception
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_turtle, "world");
            ROS_INFO("The coordinates of the point to the world are: (%.2f,%.2f,%.2f)", point_base.point.x, point_base.point.y, point_base.point.z);
        }
        catch(const std::exception& e)
        {
            ROS_INFO("Error: %s",e.what());
        }

        r.sleep();  
        ros::spinOnce();
    }

    return 0;
}