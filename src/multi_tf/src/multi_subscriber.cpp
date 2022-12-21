/*
    Implementation steps:
        1.Include header file
        2.Initialize the ros node
        3.Create the ros handle
        4.Create a TF subscription object
        5.Obtain the coordinates of the origin of son1 coordinate system in son2 from the parsing subscription information
        Resolve the coordinates of points in son1 with respect to son2
        6.spin
*/

// 1.Include header file
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

int main(int argc, char *argv[])
{   
    // 2.Initialize the ros node
    ros::init(argc,argv,"sub_frames");
    // 3.Create the ros handle
    ros::NodeHandle nh;
    // 4.Create a TF subscription object
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);
    // 5.Obtain the coordinates of the origin of son1 coordinate system in son2
    ros::Rate r(1);
    while (ros::ok())
    {
        try
        {
            // Resolve the coordinates of points in son1 with respect to son2
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("son2","son1",ros::Time(0));
            ROS_INFO("Son1 with respect to Son2: Parent coordinate system ID = %s", tfs.header.frame_id.c_str());
            ROS_INFO("Son1 with respect to Son2: Child coordinate system ID = %s", tfs.child_frame_id.c_str());
            ROS_INFO("Son1 with respect to Son2: x = %.2f, y = %.2f, z = %.2f",
                    tfs.transform.translation.x,
                    tfs.transform.translation.y,
                    tfs.transform.translation.z);

            // Coordinate point resolution
            geometry_msgs::PointStamped ps;
            ps.header.frame_id = "son1";
            ps.header.stamp = ros::Time::now();
            ps.point.x = 1.0;
            ps.point.y = 2.0;
            ps.point.z = 3.0;

            geometry_msgs::PointStamped psAtSon2;
            psAtSon2 = buffer.transform(ps, "son2");
            ROS_INFO("Coordinates in Son2: x = %.2f, y = %.2f, z = %.2f",
                    psAtSon2.point.x,
                    psAtSon2.point.y,
                    psAtSon2.point.z);
        }
        catch(const std::exception& e)
        {
            ROS_INFO("Error: %s", e.what());
        }

        r.sleep();
        // 6.spin
        ros::spinOnce();
    }
    return 0;
}