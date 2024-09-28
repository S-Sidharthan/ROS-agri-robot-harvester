#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler

#for Gripper

class Ur5GripperMoveit:

    # Constructor
    def __init__(self):


        self._planning_group = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.go(wait=True)
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


#for UR5
class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    

    def go_to_pose(self, arg_pose):                                       #go to the specified pose

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

   

    def go_to_predefined_pose(self, arg_pose_name):                                             #go to the predefined pose

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.go(wait=True)
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
        
   

    def set_joint_angles(self, arg_list_joint_angles):                                          #to set joint angles

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    rospy.sleep(8)              #delay to provide time to set up gazebo and Rviz

    ur5 = Ur5Moveit()
    ur5gripper = Ur5GripperMoveit()
    
    #defined joint angles to go and reach just before the tomato, because self path planning to the exact tomamto makes it collide with the tomatoes

    lst_joint_angles_1 = [math.radians(110),             #Join angles for 1st tomato
                          math.radians(15),
                          math.radians(-41),
                          math.radians(2),
                          math.radians(0),
                          math.radians(0)]

    lst_joint_angles_2 = [math.radians(83),              #Join angles for 2nd tomato
                          math.radians(31),
                          math.radians(-29),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]

    lst_joint_angles_3 = [math.radians(103),            #Join angles for 3rd tomato
                          math.radians(29),
                          math.radians(-4),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]



    #Exact pose of the 1st Tomato                      
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x =  -0.087356  
    ur5_pose_1.position.y =   0.239818  
    ur5_pose_1.position.z =   1.069702 
    (ur5_pose_1.orientation.x ,ur5_pose_1.orientation.y ,ur5_pose_1.orientation.z ,ur5_pose_1.orientation.w ) = quaternion_from_euler(2.704464,-0.000169,-2.799654)
    

    #Exact pose of the 2nd Tomato  
    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x =  0.110596
    ur5_pose_2.position.y =  0.375597
    ur5_pose_2.position.z =  0.958491

    (ur5_pose_2.orientation.x ,ur5_pose_2.orientation.y ,ur5_pose_2.orientation.z ,ur5_pose_2.orientation.w ) = quaternion_from_euler(-3.094235,-0.000033,3.129730)
    

    #Exact pose of the 3rd Tomato 
    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x =  -0.012659 
    ur5_pose_3.position.y =   0.287714
    ur5_pose_3.position.z =   0.682789
    (ur5_pose_3.orientation.x ,ur5_pose_3.orientation.y ,ur5_pose_3.orientation.z ,ur5_pose_3.orientation.w ) = quaternion_from_euler(-3.141505,0.000070,-2.940848)

    

    while not rospy.is_shutdown():

        #for 1st tomato
        ur5.set_joint_angles(lst_joint_angles_1)
        print("gone to pose 1 b")
        ur5.go_to_pose(ur5_pose_1)
        print("gone to pose 1")

        ur5gripper.go_to_predefined_pose("close")
        
        ur5.go_to_predefined_pose("home")

        ur5gripper.go_to_predefined_pose("open")



        #for 2nd tomato
        ur5.set_joint_angles(lst_joint_angles_2)
        print("gone to pose 2 b")
        ur5.go_to_pose(ur5_pose_2)
        print("gone to pose 2")

        ur5gripper.go_to_predefined_pose("close")

        ur5.go_to_predefined_pose("home")

        ur5gripper.go_to_predefined_pose("open")
        

        
        #for 3rd tomato
        ur5.set_joint_angles(lst_joint_angles_3)
        print("gone to pose 3 b")
        ur5.go_to_pose(ur5_pose_3)
        print("gone to pose 3")
        
        ur5gripper.go_to_predefined_pose("close")

        ur5.go_to_predefined_pose("home")

        ur5gripper.go_to_predefined_pose("open")



        rospy.signal_shutdown("Task completed successfully ") 


        

    del ur5
    del ur5gripper

if __name__ == '__main__':
    main()
