#!/usr/bin/env python
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene
from visualization_msgs.msg import Marker
# from std_msgs.msg import ColorRGBA
# from moveit_msgs.msg import ObjectColor
import sys
import rospy
import math
import moveit_commander
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R


GROUP_NAME = "xarm7"

class KyleDemo:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=0)

        self.group = moveit_commander.MoveGroupCommander(GROUP_NAME)
        self.planning_frame = self.group.get_planning_frame()
        self.envi_added = False #lab environment status
        rospy.Subscriber("/move_group/monitored_planning_scene", PlanningScene, self.planning_scene_callback)

        print("planning frame:", self.planning_frame)

        rospy.sleep(1)  # Wait for the PlanningSceneInterface to initialize

    def run(self):
        self.clear_obj()
        self.add_env()
        self.add_bench()
        self.add_box()
        self.move_cube()
        self.move_cube(pickup_pos = [0.28479, 0.63151, 0.3618], drop_pos = [0.8, -0.25, 0.29], drop_orient = [-math.sin(math.pi/2),0,0,math.cos(math.pi/2)], pickup_orient = [-math.sin(math.pi/2),0,0,math.cos(math.pi/2)])

    def normalize(arr, t_min, t_max):
        # explicit function to normalize array
        norm_arr = []
        diff = t_max - t_min
        diff_arr = max(arr) - min(arr)
        for i in arr:
            temp = (((i - min(arr))*diff)/diff_arr) + t_min
            norm_arr.append(temp)
        return norm_arr

    def clear_obj(self):
        object_names = self.scene.get_known_object_names()
        if object_names:
            print(object_names)
            for n in range (len(object_names)):
                self.scene.remove_world_object(object_names[n])
        else:
            rospy.loginfo("No objects to clear in the planning scene")
    
    def planning_scene_callback(self, data):
        # print("Callback triggered with {} collision objects.".format(len(data.world.collision_objects)))
        for obj in data.world.collision_objects:
            print("Detected object: ", obj.id)
        # Check if the object has been added to the scene
        if any(obj.id == "envi" for obj in data.world.collision_objects):
            self.envi_added = True

    def move_home(self):
        target_joint_values = [0.264, 1.4661, 1.3788, 2.2515, 0.2967, 0.8203, 0.6458, 2.2340]  # Example values
        self.group.set_joint_value_target(target_joint_values)
        plan1 = self.group.plan()
        print("planning finished")
        self.group.execute(plan1[1],wait=True)


    def add_env(self):
        # quanterion
        q_old = np.array([0.0, 1/math.sqrt(2), 0, -1/math.sqrt(2)])
        q_rot = np.array([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
        r_old = R.from_quat(q_old)
        r_rot = R.from_quat(q_rot)

        #position
        box_pos = [1.5, 2.25, -0.75]
        
        # Perform the rotation again
        r_final = r_rot * r_old

        # Convert back to quaternion
        q_final = r_final.as_quat()

        # Define the box dimensions and position
        all_pose = PoseStamped()
        all_pose.header.frame_id = "world"
        all_pose.pose.position.x = box_pos[0]
        all_pose.pose.position.y = box_pos[1]
        all_pose.pose.position.z = box_pos[2]
        all_pose.pose.orientation.x = q_final[0]
        all_pose.pose.orientation.y = q_final[1]
        all_pose.pose.orientation.z = q_final[2]
        all_pose.pose.orientation.w = q_final[3]

	# https://wiki.ros.org/rviz/DisplayTypes/Marker#Mesh_Resource_.28MESH_RESOURCE.3D10.29_.5B1.1.2B-.5D
    #     stl_file_path = "/home/kyle/catkin_ws/src/xarm_ros/xarm_gazebo/worlds/FullAssemblyVirtual_two.STL"
        stl_file_path = "package://kyle_liu_test/resources/FullAssemblyVirtual_two.STL"
        all_name = "envi"

        marker = Marker()
        # Shape (mesh resource type - 10)
        marker.type = 10
        marker.id = 0
        marker.action = 0
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.mesh_resource = stl_file_path
        marker.mesh_use_embedded_materials = True

        # Set the pose of the marker
        marker.pose.position.x = box_pos[0]
        marker.pose.position.y = box_pos[1]
        marker.pose.position.z = box_pos[2]
        # marker.pose.orientation = q_final
        marker.pose.orientation.x = q_final[0]
        marker.pose.orientation.y = q_final[1]
        marker.pose.orientation.z = q_final[2]
        marker.pose.orientation.w = q_final[3]
        
        # Scale
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        (self.scene).add_mesh(all_name, all_pose, "/home/kyle/catkin_ws_kyle/src/kyle_liu_test/kyle_liu_test/resources/FullAssemblyVirtual_two.STL")
        while not self.envi_added and not rospy.is_shutdown():
            # self.marker_pub.publish(marker)
            rospy.rostime.wallsleep(1.0)
            rospy.sleep(1)

    def add_box(self, box_pos=[0.8, -0.25, 0.26], box_orit = [0, 0, 0, 1], box_size = [0.05, 0.05, 0.05]):
        current_pose = self.group.get_current_pose().pose
        # Define the box dimensions and position
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position = current_pose.position
        box_pose.pose.position.x = box_pos[0]
        box_pose.pose.position.y = box_pos[1]
        box_pose.pose.position.z = box_pos[2]
        box_pose.pose.orientation.w = box_orit[3]
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(box_size[0], box_size[1], box_size[2]))
        rospy.sleep(1)  # Wait for the box to be added


    def add_bench(self, bench_pos=[1, -0.25, 0], bench_orit = [0, 0, 0, 1], bench_size = [0.5, 0.5, 0.5]):
        current_pose = self.group.get_current_pose().pose
        # Define the box dimensions and position
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = bench_pos[0]
        box_pose.pose.position.y = bench_pos[1]
        box_pose.pose.position.z = bench_pos[2]
        box_pose.pose.orientation.w = bench_orit[3]
        box_name = "test_bench"
        self.scene.add_box(box_name, box_pose, size=(bench_size[0], bench_size[1], bench_size[2]))
        rospy.sleep(1)  # Wait for the box to be added

    def move_cube(self, pickup_pos = [0.8, -0.25, 0.29], drop_pos = [0.28479, 0.63151, 0.3618], drop_orient = [-math.sin(math.pi/2),0,0,math.cos(math.pi/2)], pickup_orient = [-math.sin(math.pi/2),0,0,math.cos(math.pi/2)]):
        [-math.sin(math.pi/4),0,0,math.cos(math.pi/4)]
        # Define the target pose for picking up the box
        pick_pose = PoseStamped()
        pick_pose.header.frame_id = "world"
        pick_pose.pose.position.x = pickup_pos[0]
        pick_pose.pose.position.y = pickup_pos[1]
        pick_pose.pose.position.z = pickup_pos[2]
        pick_pose.pose.orientation.x = pickup_orient[0]
        pick_pose.pose.orientation.y = pickup_orient[1]
        pick_pose.pose.orientation.z = pickup_orient[2]
        pick_pose.pose.orientation.w = pickup_orient[3]

        # # Define the first location
        drop_location = PoseStamped()
        drop_location.header.frame_id = "world"
        drop_location.pose.position.x = drop_pos[0]
        drop_location.pose.position.y = drop_pos[1]
        drop_location.pose.position.z = drop_pos[2]
        drop_location.pose.orientation.x = drop_orient[0]
        drop_location.pose.orientation.y = drop_orient[1]
        drop_location.pose.orientation.z = drop_orient[2]
        drop_location.pose.orientation.w = drop_orient[3]

        # Go to box position for pickup
        self.group.set_pose_target(pick_pose)
        print("set box position done")
        self.group.set_goal_position_tolerance(0.02)

        # moveit group setting
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)
        self.group.set_planning_time(30)
        self.group.allow_replanning(True)
        plan1 = self.group.plan()
        print("picking planning finished")
        self.group.execute(plan1[1],wait=True)

        #attaching objects to robot
        self.scene.attach_box(self.group.get_end_effector_link(), "box")
        print("attach box done")

        #Moving to drop location
        self.group.set_pose_target(drop_location)
        plan = self.group.plan()
        self.group.execute(plan[1],wait = True)
    
        #detach box
        self.scene.remove_attached_object(self.group.get_end_effector_link(), "box")
        self.move_home()



if __name__ == "__main__":
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("first_test_control", anonymous=True)
        application = KyleDemo()
        application.run()
    except rospy.ROSInterruptException:
        pass
