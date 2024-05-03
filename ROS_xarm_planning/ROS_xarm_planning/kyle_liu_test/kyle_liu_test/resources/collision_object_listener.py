#!/usr/bin/env python
import rospy
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import CollisionObject
import moveit_commander
from geometry_msgs.msg import Pose, Point

# def callback(data):
#     rospy.loginfo("Received collision object data in planning scene")
#     collision_objects = data.world.collision_objects
#     if collision_objects:
#         print("collision_objects detected")
#         for object in collision_objects:
#             rospy.loginfo("object id: %s", object.id)
#             rospy.loginfo(data.get_object_poses(object.id))
#     else:
#         print("no collision objects detected")
    

def callback(data):
    rospy.loginfo("Received collision object data in planning scene")
    collision_objects = data.world.collision_objects
    if collision_objects:
        print("collision_objects detected")
        for object in collision_objects:
            rospy.loginfo("object id: %s", object.id)
            if object.primitive_poses:  # Checking if there are any poses defined
                for pose in object.primitive_poses:
                    rospy.loginfo("Pose x: %f, y: %f, z: %f", pose.position.x, pose.position.y, pose.position.z)
            else:
                rospy.loginfo("No poses available for object id: %s", object.id)
    else:
        print("no collision objects detected")


def listener():
    rospy.init_node('collision_object_listener', anonymous=True)
    rospy.Subscriber("/move_group/monitored_planning_scene", PlanningScene, callback)
    rospy.spin()  # Keep the program alive until it is killed

if __name__ == '__main__':
    listener()