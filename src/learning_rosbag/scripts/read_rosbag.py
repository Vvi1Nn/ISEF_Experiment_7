#! /usr/bin/env python3
import rospy
import rosbag
from std_msgs.msg import String

if __name__ == "__main__":
 
    rospy.init_node("w_bag_p")

    bag = rosbag.Bag("/home/zwh/experiment_4/src/learning_rosbag/data/test.bag", 'r')

    bagMessage = bag.read_messages("chatter")
    for topic,msg,t in bagMessage:
        rospy.loginfo("%s,%s,%s",topic,msg,t)

    bag.close()
