#! /usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, SpawnRequest, SpawnResponse

if __name__ == "__main__":

    rospy.init_node("turtle_spawn_p")

    client = rospy.ServiceProxy("/spawn",Spawn)

    client.wait_for_service()

    req = SpawnRequest()
    req.x = 1.0
    req.y = 1.0
    req.theta = 3.14
    req.name = "turtle2"

    try:
        response = client.call(req)
        rospy.loginfo("Success, name is: %s", response.name)
    except Exception as e:
        rospy.loginfo("Error...")