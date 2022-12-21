#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"bag_read");
    ros::NodeHandle nh;

    rosbag::Bag bag;

    bag.open("/home/zwh/experiment_4/src/learning_rosbag/data/test.bag",rosbag::BagMode::Read);

    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        std::string topic = m.getTopic();
        ros::Time time = m.getTime();
        std_msgs::String::ConstPtr p = m.instantiate<std_msgs::String>();
        if(p != nullptr)
        {
            ROS_INFO("Topic is: %s", topic.c_str());
            ROS_INFO("Time is: %.2f", time.toSec());
            ROS_INFO("Data is: %s", p->data.c_str());
        }
    }

    bag.close();
    return 0;
}