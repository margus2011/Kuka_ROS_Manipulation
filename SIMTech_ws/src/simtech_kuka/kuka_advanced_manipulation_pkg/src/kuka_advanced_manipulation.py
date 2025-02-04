#! /usr/bin/env python
import sys
import os
import copy
import rospy
import PyKDL 
import time
import moveit_commander
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import geometry_msgs.msg
from std_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import math
from sensor_msgs.msg import *
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import xml.etree.ElementTree as ET


#ROS init
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_extension', anonymous=True)

#Defines the movegroups. 
#IMPORTANT: These must be the name of these variables and move_groups. Otherwise, additional modifications of the code will be required.
robot = moveit_commander.RobotCommander() 
scene = moveit_commander.PlanningSceneInterface()
arm_left = moveit_commander.MoveGroupCommander("manipulator")
#arm_right = moveit_commander.MoveGroupCommander("arm_right")
#arms = moveit_commander.MoveGroupCommander("arms")
#torso = moveit_commander.MoveGroupCommander("torso")

arm_left.clear_pose_targets()

class EEF(object):
    """This class defines an end effector"""
    def __init__(self, EE_end_frame=PyKDL.Frame(), x=0, y=0, z=0, name="", path=""):
        """
        - EE_end_frame: Transform from the EEF base frame (the ATC part of the tool) to its actuation frame [PyKDL.Frame]
        - x: Tool size in x dimension [float]
        - y: Tool size in y dimension [float]
        - z: Tool size in z dimension. This includes the ATC part of the tool [float]
        - ATC_frame: Frame of the EEF slot in the tool changer station [PyKDL.Frame] (Removed)
        - name: Name of the tool. Name of the collision object created for the tool [string]
        - path: Path of the stl model of the tool [string]
        """
        self.EE_end_frame = EE_end_frame
        self.x = x
        self.y = y
        self.z = z
        self.name = name
        self.path = path

'''
class ATC(object):
    """
    This class manages everything related with the EEFs of the scene, including tool changing, goal poses corrections to the EEF action frames, etc.
    """
    global scene

    def __init__(self, frame_id = "base_link", left_tool = EEF(), eef_link_left = "", ATC_tools = [], left_ATC_angle=0, left_ATC_dist=0):
        """
        The constructor function initializes the tools state, adds the tools to the scene, and updates the allowed collision matrix to consider EEF collisions.
        - frame_id: Name of the parent frame used to define the ATC_frames of the EEFs [String]
        - left_tool: EEF attached to the left arm of the robot [EEF]
        - eef_link_left: Name of the link attached to the EEF of the left arm [string]
        - right_tool: EEF attached to the right arm of the robot [EEF] (Removed)
        - eef_link_right: Name of the link attached to the EEF of the right arm [string] (Removed)
        - ATC_tools: List of EEFs located initially in the ATC station [list(EEF)]
        - left_ATC_angle: Z angle between the left arm wrist and the left arm tool changer/EEF
        - right_ATC_angle: Z angle between the left arm wrist and the right arm tool changer/EEF (Removed)
        - left_ATC_dist: Length of the robot part of the left ATC
        - right_ATC_dist: Length of the robot part of the right ATC (Removed)
        """
        self.left_ATC_angle = left_ATC_angle
        self.left_ATC_dist = left_ATC_dist
        self.eef_link_left = eef_link_left

        #Remove other EEF collision objects from the scene
        attached_objects = scene.get_attached_objects()
        if len(attached_objects)>0:
                for object_name in attached_objects:
                    if "EEF" in object_name:
                        try:
                            scene.remove_attached_object(eef_link_left, name=object_name)
                            self.wait_update_object(object_name, EE_is_attached=False, EE_is_known=True)
                            rospy.sleep(0.5)
                        except:
                            pass

                scene_objects = scene.get_known_object_names()
                for object_name in scene_objects:   
                    if "EEF" in object_name:     
                        scene.remove_world_object(object_name)
                        self.wait_update_object(object_name, EE_is_attached=False, EE_is_known=False)
                        rospy.sleep(0.5)

        #Add collision objects for the EEFs
        self.EEF_left = ""
        #Dictionary with all the EEF
        self.EEF_dict = {}

        #Add Left EEF
        if left_tool.name != "":
            self.EEF_dict[left_tool.name] = left_tool
            EE_pose = geometry_msgs.msg.PoseStamped()
            EE_pose.header.frame_id = frame_id
            left_wrist_pose = arm_left.get_current_pose().pose
            EE_pose.pose = self.correctPose(left_wrist_pose, "left", ATC_sign = 1)
            self.EEF_left = left_tool.name
            try:
                scene.add_mesh(self.EEF_left, EE_pose, left_tool.path)
                self.wait_update_object(self.EEF_left, EE_is_attached=False, EE_is_known=True)
                #Attach gripper
                rospy.sleep(0.5)
                scene.attach_mesh(eef_link_left, self.EEF_left, touch_links=[eef_link_left])
                self.wait_update_object(self.EEF_left, EE_is_attached=True, EE_is_known=False)
            except:
                print("Error adding left EEF to the scene")

        #Add the rest of tools in the ATC platform
       for tool in ATC_tools:
            self.EEF_dict[tool.name] = tool
            EE_pose = geometry_msgs.msg.PoseStamped()
            EE_pose.header.frame_id = frame_id
            EE_pose.pose = frame_to_pose(tool.ATC_frame)
            try:
                scene.add_mesh(tool.name, EE_pose, tool.path)
                self.wait_update_object(tool.name, EE_is_attached=False, EE_is_known=True)
            except:
                print("Error adding tool to the scene")

        #Add attached EEFs to the initial ACM
        #Get initial ACM
        rospy.wait_for_service('/get_planning_scene')
        my_service = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        sceneReq = PlanningSceneComponents(components=128)
        sceneResponse = my_service(sceneReq)
        acm = sceneResponse.scene.allowed_collision_matrix

        rospy.wait_for_service('/apply_planning_scene')
        my_service = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        sceneReq = PlanningScene()

        #Detect index of tool changer links (wrists)
        i=0
        left_attach_link_index = -1
        for link_name in acm.entry_names:
                if link_name == eef_link_left:
                        left_attach_link_index = i       
                i+=1
        if left_attach_link_index == -1:
                print("Error")

        #Add one new column to the existig rows
        acm.entry_names.append(self.EEF_left) #-1 index
        i=0
        for val in range(len(acm.entry_values)):
                acm.entry_values[i].enabled.append(False)
                i+=1
        #Collisions are allowed just with the tool changers (adjacent to the tools)
        acm.entry_values[left_attach_link_index].enabled[-1] = True #Left tool changer - Left EEF

        #Add the values of the one new row
        entry_value_left = AllowedCollisionEntry()
        entry_value_left.enabled = []
        for link_name in acm.entry_names:
                entry_value_left.enabled.append(False)
        #Collisions are allowed just with the tool changers (adjacent to the tools)
        
        entry_value_left.enabled[left_attach_link_index] = True
        acm.entry_values.append(entry_value_left)

        #Update ACM
        sceneReq.allowed_collision_matrix = acm
        sceneReq.is_diff = True
        sceneResponse = my_service(sceneReq)

        self.ACM = acm


    def wait_update_object(self, EE_name, EE_is_known=False, EE_is_attached=False):
        """
        Wait until the object has been added/removed/attached/detached in the scene. The possible combinations are:
        - Add: EE_is_known=True, EE_is_attached=False
        - Attach: EE_is_known=True, EE_is_attached=True
        - Detach: EE_is_known=True, EE_is_attached=False
        - Remove: EE_is_known=False, EE_is_attached=False

        - EE_name: Name of the EEF collision object whose state is modified [string]
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < 2) and not rospy.is_shutdown():
                # Test if the EEF is in attached objects
                attached_objects = scene.get_attached_objects([EE_name])
                is_attached = len(attached_objects.keys()) > 0
                # Test if the EEF is in the scene.
                # Note that attaching the EEF will remove it from known_objects
                is_known = EE_name in scene.get_known_object_names()
                # Test if we are in the expected state
                if (EE_is_attached == is_attached) and (EE_is_known == is_known):
                        return True
                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time()
        return False


    def changeTool(self, new_tool, arm_side):
        """
        Manages the ATC.
        - new_tool: Name of the new tool to attach [string]
        - arm_side: Arm side in which to change the tool ["left" or "right"]
        IMPORTANT: Modify all the named target poses to match the poses defined in your SRDF file. The name of the move_groups is asumed to be: arm_left, arm_right, arms and torso.
        """
        z_offset = 0.05 #In meters
        vert_offset = 0.04 #In meters

        #Get initial ACM
        rospy.wait_for_service('/get_planning_scene')
        get_scene_service = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        getSceneReq = PlanningSceneComponents(components=128)
        sceneResponse = get_scene_service(getSceneReq)
        original_acm = sceneResponse.scene.allowed_collision_matrix
        new_acm = copy.deepcopy(original_acm)
        #Initialize servize to modify the ACM
        rospy.wait_for_service('/apply_planning_scene')
        update_scene_service = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)

        # Moving to initial position
        print("Moving robot to ATC position")
        arms.set_named_target("arms_platform_3")
        arms.go(wait=True)
        time.sleep(0.5)
        torso.set_named_target("torso_combs")
        torso.go(wait=True)
        time.sleep(0.5)

        if arm_side == "left":
                arm_right.set_named_target("arm_right_down")
                arm_right.go(wait=True)
                time.sleep(0.5)
                arm_left.set_named_target("arm_left_ATC")
                arm_left.go(wait=True)
                time.sleep(0.5)
                torso.set_named_target("torso_ATC")
                torso.go(wait=True)
                time.sleep(0.5)
                change_arm = arm_left
                current_tool = self.EEF_left
                eef_link_arm = self.eef_link_left
                touch_links_arm = [self.eef_link_left]
                ATC_dist = self.left_ATC_dist
                ATC_ang = self.left_ATC_angle
                self.EEF_left = "None"

        #Modify ACM
        #Detect index of tool that is gonna be changed
        i=0
        tool_index = -1
        for link_name in new_acm.entry_names:
                if link_name == current_tool:
                        tool_index = i     
                i+=1
        if tool_index == -1:
                print("ACM update error")
        else: #Update the tool name
                new_acm.entry_names[tool_index] = new_tool

        #Calculate trajectory keypoints
        ATC_leave_pose = frame_to_pose(self.EEF_dict[current_tool].ATC_frame)
        ATC_pick_pose = frame_to_pose(self.EEF_dict[new_tool].ATC_frame)
        ATC_leave_tool_pose_offset_approach_1 = get_shifted_pose(ATC_leave_pose, [vert_offset, 0, -z_offset - self.EEF_dict[new_tool].z, 0, 0, 0])
        ATC_leave_tool_pose_offset_approach_2 = get_shifted_pose(ATC_leave_pose, [vert_offset, 0, 0, 0, 0, 0])
        ATC_leave_tool_pose_offset_retract_1 = get_shifted_pose(ATC_leave_pose, [0, 0, -z_offset, 0, 0, 0])
        ATC_pick_tool_pose_offset_approach_1 = get_shifted_pose(ATC_pick_pose, [0, 0, -z_offset - self.EEF_dict[new_tool].z, 0, 0, 0])
        ATC_pick_tool_pose_offset_approach_2 = get_shifted_pose(ATC_pick_pose, [0, 0, -z_offset, 0, 0, 0])
        ATC_pick_tool_pose_offset_retract_1 = get_shifted_pose(ATC_pick_pose, [vert_offset, 0, 0, 0, 0, 0])
        ATC_pick_tool_pose_offset_retract_2 = get_shifted_pose(ATC_pick_pose, [vert_offset, 0, -z_offset - self.EEF_dict[new_tool].z, 0, 0, 0])

        initial_pose = change_arm.get_current_pose().pose #correct it, get the pose of the EEF base
        initial_pose_corrected_dist = get_shifted_pose(initial_pose, [0, 0, ATC_dist, 0, 0, 0])
        initial_frame_corrected = pose_to_frame(initial_pose_corrected_dist)
        initial_frame_corrected.M.DoRotZ(-ATC_ang)
        initial_pose_corrected = frame_to_pose(initial_frame_corrected)

        #Create waypoints
        success, approach_leave_waypoints = interpolate_trajectory(initial_pose = initial_pose_corrected, final_pose = ATC_leave_tool_pose_offset_approach_1, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, approach_leave_waypoints2 = interpolate_trajectory(initial_pose = ATC_leave_tool_pose_offset_approach_1, final_pose = ATC_leave_tool_pose_offset_approach_2, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, leave_tool_waypoints = interpolate_trajectory(initial_pose = ATC_leave_tool_pose_offset_approach_2, final_pose = ATC_leave_pose, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, retract_leave_waypoints = interpolate_trajectory(initial_pose = ATC_leave_pose, final_pose = ATC_leave_tool_pose_offset_retract_1, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, retract_initial_waypoints = interpolate_trajectory(initial_pose = ATC_leave_tool_pose_offset_retract_1, final_pose = initial_pose_corrected, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return

        success, approach_pick_waypoints = interpolate_trajectory(initial_pose = initial_pose_corrected, final_pose = ATC_pick_tool_pose_offset_approach_1, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, approach_pick_waypoints2 = interpolate_trajectory(initial_pose = ATC_pick_tool_pose_offset_approach_1, final_pose = ATC_pick_tool_pose_offset_approach_2, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, pick_tool_waypoints = interpolate_trajectory(initial_pose = ATC_pick_tool_pose_offset_approach_2, final_pose = ATC_pick_pose, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, retract_pick_waypoints = interpolate_trajectory(initial_pose = ATC_pick_pose, final_pose = ATC_pick_tool_pose_offset_retract_1, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, retract_pick_waypoints2 = interpolate_trajectory(initial_pose = ATC_pick_tool_pose_offset_retract_1, final_pose = ATC_pick_tool_pose_offset_retract_2, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, final_waypoints = interpolate_trajectory(initial_pose = ATC_pick_tool_pose_offset_retract_2, final_pose = initial_pose_corrected, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return

        #Change waypoints to the wrist's pose
        approach_leave_waypoints_wrist = []
        for waypoint in approach_leave_waypoints:
                approach_leave_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        approach_leave_waypoints2_wrist = []
        for waypoint in approach_leave_waypoints2:
                approach_leave_waypoints2_wrist.append(self.correctPose(waypoint, arm_side))
        leave_tool_waypoints_wrist = []
        for waypoint in leave_tool_waypoints:
                leave_tool_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        retract_leave_waypoints_wrist = []
        for waypoint in retract_leave_waypoints:
                retract_leave_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        retract_initial_waypoints_wrist = []
        for waypoint in retract_initial_waypoints:
                retract_initial_waypoints_wrist.append(self.correctPose(waypoint, arm_side))

        approach_pick_waypoints_wrist = []
        for waypoint in approach_pick_waypoints:
                approach_pick_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        approach_pick_waypoints2_wrist = []
        for waypoint in approach_pick_waypoints2:
                approach_pick_waypoints2_wrist.append(self.correctPose(waypoint, arm_side))
        pick_tool_waypoints_wrist = []
        for waypoint in pick_tool_waypoints:
                pick_tool_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        retract_pick_waypoints_wrist = []
        for waypoint in retract_pick_waypoints:
                retract_pick_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        retract_pick_waypoints2_wrist = []
        for waypoint in retract_pick_waypoints2:
                retract_pick_waypoints2_wrist.append(self.correctPose(waypoint, arm_side))
        final_waypoints_wrist = []
        for waypoint in final_waypoints:
                final_waypoints_wrist.append(self.correctPose(waypoint, arm_side))

        #Execute
        print("Approach to ATC station")
        #plan, success = compute_cartesian_path_velocity_control([approach_leave_waypoints_wrist], [20], arm_side = arm_side, max_linear_accel = 100.0)
        plan, fraction = change_arm.compute_cartesian_path(approach_leave_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Approach to ATC station 2")
        plan, success = compute_cartesian_path_velocity_control([approach_leave_waypoints2_wrist], [25], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return current_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(approach_leave_waypoints2_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Insert tool")
        plan, success = compute_cartesian_path_velocity_control([leave_tool_waypoints_wrist], [15], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return current_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(leave_tool_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Detaching gripper")
        scene.remove_attached_object(eef_link_arm, name=current_tool)
        self.wait_update_object(current_tool, EE_is_attached=False, EE_is_known=True)
        time.sleep(0.5)
        sceneReq = PlanningScene()
        sceneReq.allowed_collision_matrix = original_acm
        sceneReq.is_diff = True
        sceneResponse = update_scene_service(sceneReq) #Still allow collision between the ATC and the tool (adjacent)
        time.sleep(0.5)

        print("Retracting arm")
        plan, success = compute_cartesian_path_velocity_control([retract_leave_waypoints_wrist], [15], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return current_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(retract_leave_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Initial position")
        plan, fraction = change_arm.compute_cartesian_path(retract_initial_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Approach pick tool")
        plan, fraction = change_arm.compute_cartesian_path(approach_pick_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Approach pick tool 2")
        plan, success = compute_cartesian_path_velocity_control([approach_pick_waypoints2_wrist], [25], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return current_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(approach_pick_waypoints2_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)
        sceneReq = PlanningScene()
        sceneReq.allowed_collision_matrix = new_acm
        sceneReq.is_diff = True
        sceneResponse = update_scene_service(sceneReq) #Allow collision between ATC and the new_tool before approaching (adjacent)
        time.sleep(0.5)

        print("Pick tool")
        plan, success = compute_cartesian_path_velocity_control([pick_tool_waypoints_wrist], [15], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return current_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(pick_tool_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Attaching gripper")
        rospy.sleep(0.5)
        scene.attach_mesh(eef_link_arm, new_tool, touch_links=touch_links_arm)
        self.wait_update_object(new_tool, EE_is_attached=True, EE_is_known=False)
        sceneReq = PlanningScene()
        sceneReq.allowed_collision_matrix = new_acm
        sceneReq.is_diff = True
        sceneResponse = update_scene_service(sceneReq) #Update ACM after attaching the new tool
        self.ACM = new_acm
        time.sleep(0.5)

        print("Retracting arm")
        plan, success = compute_cartesian_path_velocity_control([retract_pick_waypoints_wrist], [15], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return new_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(retract_pick_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Retracting arm 2")
        plan, success = compute_cartesian_path_velocity_control([retract_pick_waypoints2_wrist], [25], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return new_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(retract_pick_waypoints2_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Coming back to initial ATC position")
        plan, fraction = change_arm.compute_cartesian_path(final_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        time.sleep(0.5)

        print("Moving robot to final position, ready to rotate torso")
        torso.set_named_target("torso_combs")
        torso.go(wait=True)
        time.sleep(0.5)
        arms.set_named_target("arms_platform_4")
        arms.go(wait=True)
        time.sleep(0.5)

        #Update the tool name
        if arm_side == "left":
                self.EEF_left = new_tool

        return new_tool, True


    def correctPose(self, target_pose, arm_side, rotate = False, ATC_sign = -1):
        """
        Corrects a target pose. Moveit plans the movement to the last link of the move_group, that is in the wrist. This function corrects the target pose so
        The action frame of the EEF is the one that moves to the desired target pose.
        - target_pose: Target pose for the EEF action frame [Pose]
        - arm_side: Arm side in which to change the tool ["left" or "right"]
        - rotate: True to rotate the pose 180 degrees in the EEF X axis. Useful to correct the Z axis direction of the EEF [bool]
        - ATC_sign: Determines the direction of the EEF base frame angle and distance difference with the arm wrist frame [1 or -1]
        """
        target_frame = pose_to_frame(target_pose)

        #Transform from target_pose frame to EEF action frame
        if rotate:
                target_frame.M.DoRotX(3.14) #Z axis pointing inside the tool

        #Transform from EEF action frame to EEF base frame
        if arm_side == "left":
                ATC_dist = self.left_ATC_dist
                ATC_angle = self.left_ATC_angle
                if self.EEF_left != "" and self.EEF_left != "None":
                        frame_world_EEF_base = target_frame * get_inverse_frame(self.EEF_dict[self.EEF_left].EE_end_frame)
                else:
                        frame_world_EEF_base = copy.deepcopy(target_frame)

        #Transform from EEF base frame to arm wrist
        frame_base_wrist = PyKDL.Frame()
        frame_base_wrist.p = PyKDL.Vector(0, 0, ATC_sign*ATC_dist) #Adds the offset from the tool changer to the wrist
        frame_world_wrist = frame_world_EEF_base * frame_base_wrist
        if rotate:
                frame_world_wrist.M.DoRotZ(ATC_sign*ATC_angle) #Z axis difference between the tool changer and the arm wrist
        else:
                frame_world_wrist.M.DoRotZ(-ATC_sign*ATC_angle)

        pose_world_wrist = frame_to_pose(frame_world_wrist)

        return pose_world_wrist 
'''


def frame_to_pose(frame):
        """
        Convert PyKDL.Frame into a Pose
        """
        pose_result = Pose()
        pose_result.position.x = frame.p[0] 
        pose_result.position.y = frame.p[1] 
        pose_result.position.z = frame.p[2] 
        ang = frame.M.GetQuaternion() 
        pose_result.orientation.x = ang[0] 
        pose_result.orientation.y = ang[1] 
        pose_result.orientation.z = ang[2] 
        pose_result.orientation.w = ang[3]
        return pose_result


def pose_to_frame(pose):
        """
        Converts a Pose into a PyKDL.Frame
        """
        frame_result = PyKDL.Frame() 
        frame_result.p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z) 
        frame_result.M = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return frame_result


def get_transpose_rot(rot_original):
        """
        Retrieves the transpose of a rotation matrix [PyKDL.Rotation]
        """
        rot_trans = PyKDL.Rotation(rot_original[0,0], rot_original[1,0], rot_original[2,0], rot_original[0,1], rot_original[1,1], rot_original[2,1], rot_original[0,2], rot_original[1,2], rot_original[2,2])
        return rot_trans


def get_inverse_frame(frame_original):
        """
        Retrieves the inverse of a frame [PyKDL.Frame]
        """
        frame_inv = PyKDL.Frame() 
        x = -(frame_original.p[0]*frame_original.M[0,0] + frame_original.p[1]*frame_original.M[1,0] + frame_original.p[2]*frame_original.M[2,0])
        y = -(frame_original.p[0]*frame_original.M[0,1] + frame_original.p[1]*frame_original.M[1,1] + frame_original.p[2]*frame_original.M[2,1])
        z = -(frame_original.p[0]*frame_original.M[0,2] + frame_original.p[1]*frame_original.M[1,2] + frame_original.p[2]*frame_original.M[2,2])
        frame_inv.p = PyKDL.Vector(x,y,z) 
        frame_inv.M = get_transpose_rot(frame_original.M)
        return frame_inv


def compute_distance(pose1, pose2):
    """
    Retrieves the linear distance [mm] between two poses.
    """
    dist = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)
    return dist
   

def compute_angle_distance(pose1, pose2):
    """
    Retrieves the angular [rad] distance between two poses.
    """
    frame1 = pose_to_frame(pose1)
    frame2 = pose_to_frame(pose2)
    frame12 = frame1.Inverse() * frame2
    rot = abs(frame12.M.GetRotAngle()[0])
    return rot


def compute_lin_or_ang_distance(pose1, pose2, linear = True):
    """
    Retrieves the linear (linear = True) distance [mm] or angular (linear = False) [rad] distance between two poses.
    """
    if linear:
        return compute_distance(pose1, pose2)
    else:
        return compute_angle_distance(pose1, pose2)


def get_shifted_pose(origin_pose, shift):
        """
        Retrives a shifted pose
        - origin_pose: Original pose [Pose]
        - shift: List of shifts and rotations in all the angles ([x displacement, y displacement, z displacement, x rotation, y rotation, z rotation]). Dist in m and angles in rad
        """
        tf_origin = pose_to_frame(origin_pose)   
        tf_shift = PyKDL.Frame() 
        tf_shift.p = PyKDL.Vector(shift[0], shift[1], shift[2]) 
        tf_shift.M.DoRotX(shift[3]) 
        tf_shift.M.DoRotY(shift[4]) 
        tf_shift.M.DoRotZ(shift[5])
        tf_result = tf_origin * tf_shift

        pose_result = frame_to_pose(tf_result)
        return pose_result


def degree_difference(R1, R2):
        """
        Retrieves the angle difference between two PyKDL.Rotation
        """
        R_1_2 = get_transpose_rot(R1) * R2
        rad_dif = R_1_2.GetRPY()
        deg_dif = [element * (180.0/math.pi) for element in rad_dif]
        return deg_dif, rad_dif


def interpolate_trajectory(initial_pose, final_pose, step_pos_min, step_deg_min, n_points_max):
        """
        Creates several waypoints between two poses:
        - initial_pose: Initial pose [Pose]
        - final_pose: Final pose [Pose]
        - step_pos_min: Minimum distance between consecutive intermediate poses [m]
        - step_deg_min: Minimum angle between consecutive intermediate poses [angle]
        - n_points_max: Maximum number of waypoints of the interpolated path [int]
        """
        waypoints = []
        if initial_pose == final_pose:
                print("Cannot interpolate points, it is the same pose")
                return False, waypoints

        n_points = n_points_max
        step_pos_min = float(step_pos_min)
        step_deg_min = float(step_deg_min)
        pos_dif = compute_distance(initial_pose, final_pose)
        deg_dif, rad_dif = degree_difference(pose_to_frame(initial_pose).M, pose_to_frame(final_pose).M)
        n_points_list = []
        n_points_list.append(pos_dif/step_pos_min)
        n_points_list.append(abs(deg_dif[0])/step_deg_min)
        n_points_list.append(abs(deg_dif[1])/step_deg_min)
        n_points_list.append(abs(deg_dif[2])/step_deg_min)
        if max(n_points_list) < 20:
                n_points = int(max(n_points_list))
        
        waypoints.append(initial_pose)
        for point in range(n_points):
            if point > 0:
                x = initial_pose.position.x + ((final_pose.position.x - initial_pose.position.x)*float(point)/float(n_points))
                y = initial_pose.position.y + ((final_pose.position.y - initial_pose.position.y)*float(point)/float(n_points))
                z = initial_pose.position.z + ((final_pose.position.z - initial_pose.position.z)*float(point)/float(n_points))
                rotation = pose_to_frame(initial_pose).M
                rotation.DoRotX((rad_dif[0])*float(point)/float(n_points))
                rotation.DoRotY((rad_dif[1])*float(point)/float(n_points))
                rotation.DoRotZ((rad_dif[2])*float(point)/float(n_points))
                new_frame = PyKDL.Frame() 
                new_frame.p = PyKDL.Vector(x,y,z) 
                new_frame.M = rotation
                waypoints.append(frame_to_pose(new_frame))
        waypoints.append(final_pose)

        return True, waypoints             


def adjust_plan_speed(traj_poses, EE_speed_aux, v_change, traj_mov, max_accel, all_plans, linear = True):
    """
    Recalculate the times of an EEF trajectory to follow some target speeds.
    This function is just called internally by another function.
    - traj_poses: List of lists of EEF poses. Each sublist corresponds to the poses of a different target speed plan
    - EE_speed_aux: List of target speeds (mm/s or deg/s)
    - v_change: Dictionary with the distance needed to accelerate between succesive speed sections (mm/s or deg/s)
    - traj_mov: List of total distance/angle travelled in each speed section (mm or deg)
    - max_accel: Maximum EEF acceleration (mm/s^2 or rad/s^2)
    - all_plans: All the plans of the different speed sections without EEF speed control
    - linear: True for linear motion, False for angular motion
    """

    #Variables initialization
    thres = 0.05
    if not linear:
        thres *= (0.7*(math.pi/180))
    corrected_traj = []
    zero_Jvel = []
    for i in range(len(all_plans[0].joint_trajectory.points[0].positions)):
        zero_Jvel.append(0.0)
    corrected_traj.append({'pose': traj_poses[0][0], 'state': all_plans[0].joint_trajectory.points[0].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': 0, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})

    success = True
    i=0
    s_i = 1
    v_chg_i = 0
    init_speed_change = False
    final_speed_change = False
    for plan_poses in traj_poses:
            if traj_mov[i] < thres: #No movement so the plan is skipped
                j=-1
                for pose in plan_poses:
                    j+=1
                    if pose == corrected_traj[-1]['pose']:
                            continue
                    try:
                        corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                    except:
                        pass 
                continue

            init_speed_change = EE_speed_aux[s_i] > EE_speed_aux[s_i-1] #Transition at the beginning if the previous velocity was lower
            final_speed_change = EE_speed_aux[s_i] > EE_speed_aux[s_i+1] #Transition at the end if the next velocity is higher
            j=-1
            x_plan = 0
            init_transition_indexes = []
            first_final = True
            for pose in plan_poses:
                    j+=1
                    if pose == corrected_traj[-1]['pose']: #Repeated point, skipped
                            continue
                    
                    if compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) < thres and not init_speed_change:
                            corrected_traj.append({'pose': corrected_traj[-1]['pose'], 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                            continue
                    
                    #Acceleration/decelarion at the beginning of the current speed section
                    if init_speed_change:
                            if final_speed_change:
                                if (v_change[v_chg_i]['x_min_req'] + v_change[v_chg_i+1]['x_min_req']) > traj_mov[i]: #It is not possible to reach the target speed and decelerate on time
                                    t2= (-2*EE_speed_aux[s_i+1] + math.sqrt(2*(EE_speed_aux[s_i+1]**2 + EE_speed_aux[s_i-1]**2 + 2*max_accel*traj_mov[i])))/(2*max_accel)
                                    v_change[v_chg_i+1]['x_min_req'] = (EE_speed_aux[s_i+1] + max_accel*t2)*t2 - (0.5 * max_accel * t2**2)
                                    v_change[v_chg_i]['x_min_req'] = traj_mov[i] - v_change[v_chg_i+1]['x_min_req']
                            x_plan += compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) #Distance travelled until the moment
                            #Adds the new configuration with empty speed, accel and time, which will be calculated later
                            corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': 0, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                            init_transition_indexes.append(len(corrected_traj)-1)

                            #When the distance travelled until the moment is higher than the required distance for the initial acceleration, the speed, acceleration and time for all these points is calculated
                            if (x_plan + compute_lin_or_ang_distance(pose, plan_poses[min(j+1,len(plan_poses))], linear)) > v_change[v_chg_i]['x_min_req'] or (traj_mov[i] < v_change[v_chg_i]['x_min_req'] and j >= (len(plan_poses)-1)): #Determine the real discrete point where the speed change is completed
                                    speed_diff = EE_speed_aux[s_i] - EE_speed_aux[s_i-1] #Speed difference between consecutive target speed sections
                                    trans_accel = min((2*speed_diff*EE_speed_aux[s_i-1] + speed_diff**2)/(2*x_plan),max_accel) #Acceleration is constant as we are considering an u.a.r.m.
                                    for index in init_transition_indexes:
                                            #Apply the equations of a trapezoidal speed profile to calculate the times during the acceleration period
                                            corrected_traj[index]['EE_accel'] = trans_accel
                                            #Calculate the time difference between consecutive points
                                            t_A = trans_accel/2
                                            t_B = corrected_traj[index-1]['EE_speed']
                                            t_C = -compute_lin_or_ang_distance(corrected_traj[index]['pose'], corrected_traj[index-1]['pose'], linear)
                                            new_time_1 = (-t_B + math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                            new_time_2 = (-t_B - math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                            if new_time_1 < 0 and new_time_2 < 0:
                                                new_time = 0
                                                success = False
                                            elif new_time_1 < 0:
                                                new_time = new_time_2
                                            elif new_time_2 < 0:
                                                new_time = new_time_1
                                            elif new_time_1 < new_time_2:
                                                new_time = new_time_1
                                            else:
                                                new_time = new_time_2
                                            corrected_traj[index]['time'] = corrected_traj[index-1]['time'] + new_time
                                            corrected_traj[index]['EE_speed'] = corrected_traj[index-1]['EE_speed'] + trans_accel*new_time #Calculate the EEF speed reached in the next point
                                    corrected_traj[-1]['EE_accel'] = 0 #Finish the movement with accel 0, velocity will be constant after the transition
                                    corrected_traj[init_transition_indexes[0]-1]['EE_accel'] = trans_accel #Start the movement accelerating
                                    v_chg_i += 1
                                    init_speed_change = False

                    #Acceleration/decelarion at the end of the current speed section
                    elif final_speed_change:
                            x_plan += compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear)
                            x_left = traj_mov[i] - x_plan #Remaining distance to travel before the end of the current speed section
                            #When the remaining distance is lower than the minimum distance required for the final acceleration/deceleration, it must start from the prev point
                            if x_left < v_change[v_chg_i]['x_min_req']: 
                                    if first_final:
                                            x_trans = x_left + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) #The prev x
                                            speed_diff = EE_speed_aux[s_i+1] - corrected_traj[-1]['EE_speed']
                                            trans_accel_1 = (2*speed_diff*EE_speed_aux[s_i] + speed_diff**2)/(2*x_trans)
                                            trans_accel_2 = copy.deepcopy(-max_accel)
                                            if abs(trans_accel_1)>abs(trans_accel_2):
                                                trans_accel = trans_accel_2
                                            else:
                                                trans_accel = trans_accel_1
                                            first_final = False
                                            corrected_traj[-1]['EE_accel'] = trans_accel

                                    if speed_diff == 0:
                                        corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                                        continue

                                    #Apply the equations of a trapezoidal speed profile to calculate the times during the acceleration/deceleration period
                                    t_A = trans_accel/2
                                    t_B = corrected_traj[-1]['EE_speed']
                                    t_C = -compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear)
                                    t_shorter = False
                                    
                                    if (t_B**2 - 4*t_A*t_C) <= 0 or t_shorter:
                                        new_time = (-t_B)/(2*t_A)
                                    else:
                                        new_time_1 = (-t_B + math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                        new_time_2 = (-t_B - math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                        if new_time_1 < 0 and new_time_2 < 0:
                                            new_time = 0
                                            success = False
                                        elif new_time_1 < 0:
                                            new_time = new_time_2
                                        elif new_time_2 < 0:
                                            new_time = new_time_1
                                        elif new_time_1 < new_time_2:
                                            new_time = new_time_1
                                        else:
                                            new_time = new_time_2
                                        if new_time > corrected_traj[-1]['time'] - corrected_traj[-2]['time']:
                                            pass
                                        else:
                                            t_shorter = False

                                    new_total_time = corrected_traj[-1]['time'] + new_time
                                    new_speed = corrected_traj[-1]['EE_speed'] + trans_accel*new_time

                                    corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': new_speed, 'EE_accel': trans_accel, 'time': new_total_time, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                                    if (linear and x_left < 0.1) or (not linear and x_left < 0.07*(math.pi/180)): #Last point
                                            corrected_traj[-1]['EE_accel'] = 0    
                                            v_chg_i += 1
                                            final_speed_change = False

                            #When there is still time before the final acceleration/deceleration the speed is kept constant at the target speed and the acceleration is 0                
                            else:
                                    corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': EE_speed_aux[s_i], 'EE_accel': 0, 'time': corrected_traj[-1]['time'] + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'],linear)/EE_speed_aux[s_i], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)}) 

                    #If there are no target speed changes between sections, or the next target speed is higher than the current, the speed is kept constant until the end of the plan
                    else:
                            corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': EE_speed_aux[s_i], 'EE_accel': 0, 'time': corrected_traj[-1]['time'] + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'],linear)/EE_speed_aux[s_i], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})   

            i+=1
            s_i += 1
    return corrected_traj, success


def compute_cartesian_path_velocity_control(waypoints_list, EE_speed, EE_ang_speed = [], arm_side = "left", max_linear_accel = 200.0, max_ang_accel = 140.0, extra_info = False, step = 0.002):
        """
        Function that generates a single-arm motion plan with control on the EEF speed, ready to be executed.
        - waypoints_list: List of lists of waypoints [Poses]. Every sublist has a different EEF target speed associated
        - EE_speed: The list of EEF target linear speeds for each speed section of the trajectory [mm/s]
        - EE_ang_speed: The List of EEF target angular speeds for each speed section of the trajectory [deg/s]
        - max_linear_accel: Maximum linear acceleration [mm/s^2]
        - max_angular_accel: Maximum linear acceleration [deg/s^2]
        - extra_info: Used for the sync policy 3 dual-arm function. Leave it as False otherwise
        - Step: Step used for the compute_cartesian_path function [mm]
        """
        
        success = True
        #Selects the arm to be planned
        if arm_side == "left":
            arm = arm_left
            fkln = ['link_6'] #Modify with the name of the final link of the left arm
        if len(EE_ang_speed)==0: #If the angular speed limit is not specified, it considers a limit of 1 mm/s --> 0.7 deg/s
            for s in EE_speed:
                EE_ang_speed.append(s*0.7)

        #Convert to rads
        for i in range(len(EE_speed)):
            EE_ang_speed[i]*=(math.pi/180)

        max_ang_accel *= (math.pi/180)

        #Define the speed profile accelerations
        EE_speed_aux = copy.deepcopy(EE_speed)
        EE_speed_aux.insert(0,0)
        EE_speed_aux.append(0)
        EE_ang_speed_aux = copy.deepcopy(EE_ang_speed)
        EE_ang_speed_aux.insert(0,0)
        EE_ang_speed_aux.append(0)
        v_change = []
        v_change_ang = []

        #Calculates the minimum distance required to accelerate and/or decelerate the EEF between the different speed sections
        for i in range(len(EE_speed_aux)-1):
                max_linear_accel_sign = copy.deepcopy(max_linear_accel)
                if EE_speed_aux[i]>EE_speed_aux[i+1]:
                    max_linear_accel_sign *= -1
                max_ang_accel_sign = copy.deepcopy(max_ang_accel)
                if EE_ang_speed_aux[i]>EE_ang_speed_aux[i+1]:
                    max_ang_accel_sign *= -1
                
                t_req_lin = (EE_speed_aux[i+1]-EE_speed_aux[i])/max_linear_accel_sign
                t_req_ang = (EE_ang_speed_aux[i+1]-EE_ang_speed_aux[i])/max_ang_accel_sign
                change = {'t_req': t_req_lin, 'x_min_req': EE_speed_aux[i]*t_req_lin + (max_linear_accel_sign*(t_req_lin**2))/2}
                change_ang = {'t_req': t_req_ang, 'x_min_req': EE_ang_speed_aux[i]*t_req_ang + (max_ang_accel_sign*(t_req_ang**2))/2}

                v_change.append(change)
                v_change_ang.append(change_ang)

        #Plan the trajectory of every speed section using the compute_cartesian_path function or a custom IK trajectory solver.
        #This plan don't have any control over the speed of the end effector
        all_plans = []
        for traj in waypoints_list:
                (plan, fraction) = arm.compute_cartesian_path(traj, step, 0.0)
                all_plans.append(plan)
                rs = RobotState()
                for j_name in plan.joint_trajectory.joint_names:
                        rs.joint_state.name.append(j_name)
                for state in plan.joint_trajectory.points[-1].positions:
                        rs.joint_state.position.append(state)        
                arm.set_start_state(rs)
        arm.set_start_state_to_current_state()

        #Get all the EEF poses of the generated plan using a forward kinematics solver
        traj_poses = []
        traj_mov_position = []
        traj_mov_angle = []
        rospy.wait_for_service('compute_fk')
        fk_srv = rospy.ServiceProxy('compute_fk', GetPositionFK)
        rs = RobotState()
        for j_name in plan.joint_trajectory.joint_names:
                rs.joint_state.name.append(j_name)      
        for plan in all_plans:
                plan_poses = []
                traj_mov_i_position = 0
                traj_mov_i_angle = 0
                for joint_state in plan.joint_trajectory.points:
                        rs.joint_state.position = []
                        for joint in joint_state.positions:
                                rs.joint_state.position.append(joint)              
                        header = Header(0,rospy.Time.now(),"base_link") #Modify with the frame_id of the robot
                        header.frame_id = plan.joint_trajectory.header.frame_id
                        plan_pose_meters = fk_srv(header, fkln, rs).pose_stamped[0].pose
                        plan_pose_mm = copy.deepcopy(plan_pose_meters)
                        plan_pose_mm.position.x *= 1000
                        plan_pose_mm.position.y *= 1000
                        plan_pose_mm.position.z *= 1000
                        plan_poses.append(plan_pose_mm)                             
                        if len(plan_poses) > 1:
                                traj_mov_i_position += compute_distance(plan_poses[-2], plan_poses[-1])
                                traj_mov_i_angle += compute_angle_distance(plan_poses[-2], plan_poses[-1])
                traj_poses.append(plan_poses)
                traj_mov_position.append(traj_mov_i_position) #Total travelled distance in each speed section
                traj_mov_angle.append(traj_mov_i_angle) #Total travelled angle in each speed section

        #Get max joint velocities from the URDF robot description
        robot_desc = rospy.get_param('robot_description')
        root = ET.fromstring(robot_desc)
                
        vel_limit = {}
        for child in root: 
                if child.tag == "joint":
                        j_name = child.get("name")
                        if child.get("type") == "revolute":
                                for joint_attrib in child:
                                        if joint_attrib.tag == "limit":
                                                vel_limit[j_name] = float(joint_attrib.get("velocity"))*0.9 #The limit is set at a 90% of the URDF limit

        #Recalculate all the times of the trajectory according to the specified target speed profiles. 
        #The calculation is done both for the linear and the angular movements
        corrected_traj, success_lin = adjust_plan_speed(traj_poses, EE_speed_aux, v_change, traj_mov_position, max_linear_accel, all_plans, linear = True)
        corrected_traj_ang, success_ang = adjust_plan_speed(traj_poses, EE_ang_speed_aux, v_change_ang, traj_mov_angle, max_ang_accel, all_plans, linear = False)
        if not success_lin or not success_ang:
            success = False   

        #Merges the linear and angular constraint plans by selecting the larger times, to not exceed any of the limits.
        first_accel = True
        full_corrected_traj = copy.deepcopy(corrected_traj)
        for i in range(len(corrected_traj)-1):
            try:
                if (corrected_traj_ang[i+1]['time'] - corrected_traj_ang[i]['time']) > (corrected_traj[i+1]['time'] - corrected_traj[i]['time']):
                        full_corrected_traj[i+1]['time'] = full_corrected_traj[i]['time'] + (corrected_traj_ang[i+1]['time'] - corrected_traj_ang[i]['time'])
                else:
                        full_corrected_traj[i+1]['time'] = full_corrected_traj[i]['time'] + (corrected_traj[i+1]['time'] - corrected_traj[i]['time'])
            except:
                pass
            if extra_info:
                if corrected_traj[i+1]['EE_accel'] == 0 and corrected_traj_ang[i+1]['EE_accel'] == 0 and first_accel:
                        first_accel = False
                        t_accel = full_corrected_traj[i+1]['time']
                if corrected_traj[i+1]['EE_accel'] == 0 and corrected_traj_ang[i+1]['EE_accel'] == 0 and ((i+1)<(len(corrected_traj)-1)):
                        t_dec = full_corrected_traj[i+1]['time']

        zero_Jvel = []
        for i in range(len(all_plans[0].joint_trajectory.points[0].positions)):
                zero_Jvel.append(0.0)

        #Recalculate the times of the trajectory when any of the joint velocity limits are exceeded (> 90% of limit)
        update_time = False
        full_corrected_traj_with_limits = copy.deepcopy(full_corrected_traj)
        for i in range(len(full_corrected_traj)-1):
                time_diff = full_corrected_traj[i+1]['time']-full_corrected_traj[i]['time']
                updated_new_times = []
                for j in range(len(full_corrected_traj[i]['state'])):
                        if time_diff != 0:
                            angle_diff = full_corrected_traj[i+1]['state'][j]-full_corrected_traj[i]['state'][j]
                            new_Jaccel = (angle_diff-(full_corrected_traj_with_limits[i]['Jspeed'][j]*time_diff))*(2/(time_diff**2))
                            new_Jspeed = full_corrected_traj_with_limits[i]['Jspeed'][j] + (new_Jaccel*time_diff)
                            new_time = full_corrected_traj_with_limits[i]['time'] + time_diff

                        joint_name = rs.joint_state.name[j]
                        if vel_limit[joint_name] < new_Jspeed or time_diff == 0:
                            new_Jspeed = vel_limit[joint_name]
                            new_Jaccel = ((2*full_corrected_traj_with_limits[i]['Jspeed'][j]*(new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])) + (new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])**2)/(2*angle_diff)
                            if abs(new_Jaccel) < 0.0001:
                                new_time_step = (angle_diff)/new_Jspeed
                            else:
                                new_time_step = (new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])/new_Jaccel
                            updated_new_times.append(new_time_step)
                            update_time = True

                        full_corrected_traj_with_limits[i+1]['time'] = new_time
                        full_corrected_traj_with_limits[i+1]['Jaccel'][j] = new_Jaccel
                        full_corrected_traj_with_limits[i+1]['Jspeed'][j] = new_Jspeed

                if update_time: #If any of the limits is exceeded, recalculate times, joint velocities and accelerations
                    print("Joint limits speed exceeded")
                    update_time = False
                    new_time_max = max(updated_new_times)
                    time_diff = new_time_max
                    full_corrected_traj_with_limits[i+1]['time'] = full_corrected_traj_with_limits[i]['time'] + time_diff
                    for j in range(len(full_corrected_traj[i]['state'])):
                        angle_diff = full_corrected_traj[i+1]['state'][j]-full_corrected_traj[i]['state'][j]
                        new_Jaccel = (angle_diff-(full_corrected_traj_with_limits[i]['Jspeed'][j]*time_diff))*(2/(time_diff**2))
                        new_Jspeed = full_corrected_traj_with_limits[i]['Jspeed'][j] + (new_Jaccel*time_diff)
                        full_corrected_traj_with_limits[i+1]['Jaccel'][j] = new_Jaccel
                        full_corrected_traj_with_limits[i+1]['Jspeed'][j] = new_Jspeed

                if extra_info and t_accel == full_corrected_traj[i+1]['time']:
                    t_accel = full_corrected_traj_with_limits[i+1]['time']
                if extra_info and t_dec == full_corrected_traj[i+1]['time']:
                    t_dec = full_corrected_traj_with_limits[i+1]['time']

        full_corrected_traj_with_limits[-1]['Jspeed'] = copy.deepcopy(zero_Jvel)

        #Uncomment if we want to visualize all the information of the generated plan
        '''
        i=0
        for traj_point in full_corrected_traj_with_limits:
                print(i)
                print("Position: " + str(traj_point['pose'].position.x) + ", " + str(traj_point['pose'].position.y) + ", " + str(traj_point['pose'].position.z))
                print("Velocity: " + str(traj_point['EE_speed']))
                print("Acceleration: " + str (traj_point['EE_accel']))
                print("Time: " + str (traj_point['time']))
                print("Joint angles: " + str (traj_point['state']))
                print("Joint velocities: " + str (traj_point['Jspeed']))
                print("Joint accelerations: " + str (traj_point['Jaccel']))
                print("----------------------")
                i+=1
        '''

        #Adjust the generated plan to the RobotTrajectory() msg structure
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header.frame_id = "base_link"
        new_plan.joint_trajectory.joint_names = copy.deepcopy(rs.joint_state.name) 
        for state in full_corrected_traj_with_limits:
                point = JointTrajectoryPoint()
                point.positions = copy.deepcopy(state['state'])
                point.velocities = copy.deepcopy(state['Jspeed'])
                point.accelerations = copy.deepcopy(state['Jaccel'])
                point.effort = []
                point.time_from_start.secs = int(copy.deepcopy(state['time']))
                point.time_from_start.nsecs = int((copy.deepcopy(state['time']) - int(copy.deepcopy(state['time'])))*1000000000)
                new_plan.joint_trajectory.points.append(point)

        if extra_info:
                return new_plan, success, t_accel, t_dec
        else:
                return new_plan, success         
        
def user_defined_cartesian(coordinates):
        ### Coordinates - Refers to a list containing coordinates of the points [x,y,z]
        current_pose = arm_left.get_current_pose().pose
        new_pose_left = copy.deepcopy(current_pose)
        new_pose_left.position.x += float(coordinates[0]*0.001)
        new_pose_left.position.y += float(coordinates[1]*0.001)
        new_pose_left.position.z += float(coordinates[2]*0.001) 
        new_pose_left.orientation.x += float(coordinates[3])      
        new_pose_left.orientation.y += float(coordinates[4])
        new_pose_left.orientation.z += float(coordinates[5])

        #Speed profile
        plan, success = compute_cartesian_path_velocity_control([[current_pose, new_pose_left]], [30.0])
        if success:
                arm_left.execute(plan, wait=True)
        time.sleep(0.5)
        print("Executed custom coordinates -",coordinates)

def origin_defined_cartesian(coordinates,pose):
        ### Coordinates - Refers to a list containing coordinates of the points [x,y,z]
        new_pose_left = copy.deepcopy(pose)
        new_pose_left.position.x += float(coordinates[0]*0.001)
        new_pose_left.position.y += float(coordinates[1]*0.001)
        new_pose_left.position.z += float(coordinates[2]*0.001) 
        new_pose_left.orientation.x += float(coordinates[3]*0.001)      
        new_pose_left.orientation.y += float(coordinates[4]*0.001)
        new_pose_left.orientation.z += float(coordinates[5]*0.001)

        #Speed profile
        plan, success = compute_cartesian_path_velocity_control([[pose, new_pose_left]], [20.0])
        if success:
                arm_left.execute(plan, wait=True)
        time.sleep(0.5)
        print("Executed custom coordinates -",coordinates)

def user_cartesian_cont(cords):
        origin=poses=[]
        current_pose = arm_left.get_current_pose().pose        
        for i in range(len(cords)):
                if i!=0:
                        delta=[]
                        for j in range(3):
                                delta.append((cords[i][j]-origin[j])*0.001)
                        new_pose = copy.deepcopy(current_pose)
                        new_pose.position.x += float(delta[0])
                        new_pose.position.y += float(delta[1])
                        new_pose.position.z += float(delta[2])
                        poses.append(new_pose)
                else:
                        origin=cords[i]
                        new_pose_left = copy.deepcopy(current_pose)
                        poses.append(new_pose_left)
        plan, success = compute_cartesian_path_velocity_control([poses], [30.0])
        if success:
                arm_left.execute(plan, wait=True)
        time.sleep(0.5)
        print("Executed Set of Coordinates")                

def get_joint_state():
    msg=rospy.wait_for_message("/joint_states", JointState, timeout=None)
    print("Received joint states")
    # Define a one-time callback inside the function
    def joint_state_callback(msg):
        a=[]
        for i, name in enumerate(msg.name):
            position_in_radians = msg.position[i]  # Assuming positions are in radians
            #print(f"Joint {name}: Position in radians: {position_in_radians}")
            a.append(position_in_radians)
        return a
    return joint_state_callback(msg)
    rospy.sleep(1)  # Just to make sure the callback has time to log information
                
                      

######################################################################################################################################

if __name__ == '__main__':
        #IMPORTANT: Modify all the named target poses to match the poses defined in your SRDF file
        #Define parameters of the tools for the ATC objects
        eef_link_left = "link_6"
        touch_links_left = ["link_6"]

        gripper_end_frame = PyKDL.Frame() 
        gripper_end_frame.p = PyKDL.Vector(0, 0, 0.3) 

        EE_file_path_gripper = os.path.join(os.path.dirname(__file__), '../../simtech_kuka/simtech_kuka_workcell/meshes/spindle/visual/spindle.stl')

        gripper_left = EEF(EE_end_frame = gripper_end_frame, x = 0.246, y = 0.7048, z = 0.35, name = "EEF_gripper_left", path = EE_file_path_gripper)
        
        #user_defined_cartesian([0,0,0.005,0,0,0])

        print("Moving arms to initial position")
        #arm_left.set_named_target("origin-position")
        arm_left.set_joint_value_target(get_joint_state())
        arm_left.go(wait=True)
        print("Moved to Initial Position")
        time.sleep(0.5)

        #user_defined_cartesian([-0.3,0,0.0,0,0,0])

        with open('/home/jeeva/catkin_ws/src/GitHub/SIMTech_ws/src/industrial_robot_ros_packages/simtech_kuka/kuka_advanced_manipulation_pkg/src/data_text.txt','r') as file:   
                cords=[]
                am_cords=[]
                new_motion_index=[]
                for j in range(500):
                        i=file.readline()
                        splitted=i.split()
                        cords=[float(x[1:]) for x in splitted[1:4]]
                        am_cords.append(cords)
                for i in range(1,len(am_cords)):
                        if abs(am_cords[i][1]-am_cords[i-1][1]) >= 5:
                                new_motion_index.append(i-1)
                list_len=[]
                for motion in range(len(new_motion_index)):
                        if motion == 0:
                                user_cartesian_cont(am_cords[:new_motion_index[motion]+1])
                                user_defined_cartesian([xi - yi for xi, yi in zip(am_cords[new_motion_index[motion]+1], am_cords[new_motion_index[motion]])]+[0,0,0])
                                #print([xi - yi for xi, yi in zip(am_cords[new_motion_index[motion]+1], am_cords[new_motion_index[motion]])]+[0,0,0])
                        else:
                                #pass
                                user_cartesian_cont(am_cords[new_motion_index[motion-1]+1:new_motion_index[motion]+1])
                                user_defined_cartesian([xi - yi for xi, yi in zip(am_cords[new_motion_index[motion]+1], am_cords[new_motion_index[motion]])]+[0,0,0])

                        #print([xi - yi for xi, yi in zip(am_cords[new_motion_index[motion]+1], am_cords[new_motion_index[motion]])]+[0,0,0])
                        #list_len.append(len(am_cords[new_motion_index[motion-1]+1:new_motion_index[motion]+1]))
                user_cartesian_cont(am_cords[new_motion_index[len(new_motion_index)-1]+1:])


'''
        with open('/home/jeeva/catkin_ws/src/GitHub/SIMTech_ws/src/industrial_robot_ros_packages/simtech_kuka/kuka_advanced_manipulation_pkg/src/data_text.txt','r') as file:   
                cords=[]
                am_cords=[]
                for j in range(200):
                        i=file.readline()
                        splitted=i.split()
                        cords=[float(x[1:]) for x in splitted[1:4]]
                        am_cords.append(cords)
                user_cartesian_cont(am_cords)  
   '''      
        #user_defined_cartesian([-1,-1,2,0,0,0])
        #user_defined_cartesian([-0.3,0,0.0,0,0,0])



'''
        with open('/home/jeeva/catkin_ws/src/GitHub/SIMTech_ws/src/industrial_robot_ros_packages/simtech_kuka/kuka_advanced_manipulation_pkg/src/data_text.txt','r') as file:   
                origin=True
                prev_delta=new_delta=[]
                for j in range(20):
                        i=file.readline()
                        splitted=i.split()
                        cords=[float(x[1:]) for x in splitted[1:4]]
                        if not origin:
                                delta=[]
                                for a in range(3):
                                        delta.append((cords[a]-origin_val[a])*0.01)
                                delta+=[0,0,0]
                                if j>=2:
                                      new_delta=delta
                                      dist=[]
                                      for x in range(3):
                                            dist.append(new_delta[x]-prev_delta[x])
                                      dist+=[0,0,0]      
                                      user_defined_cartesian(dist)
                                      prev_delta=new_delta
                                elif j==1:
                                      prev_delta=delta      
                        else:
                                origin_val=cords
                                origin=False
'''
'''    
        user_defined_cartesian([-2,0,2,0,0,0]) 
        #cords=eval(input("enter coordinates - "))
        user_defined_cartesian([-1,0,-1,0,0,0])
  
'''