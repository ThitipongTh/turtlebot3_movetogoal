#!/usr/bin/python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import tf.transformations

class moveBaseAction():
    def __init__(self):
        rospy.loginfo("Wait for amcl")
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amclCallback)
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def amclCallback(self, msg):
        robot_pose_ = msg.pose.pose
        robot_theta_ = tf.transformations.euler_from_quaternion((robot_pose_.orientation.x, robot_pose_.orientation.y, robot_pose_.orientation.z, robot_pose_.orientation.w))
        rospy.loginfo(f"Robot Pose=> x:{robot_pose_.position.x} y:{robot_pose_.position.y} theta:{robot_theta_[2]}")
    
    def createGoal(self, x:float, y:float, theta:float):
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        
        return goal
    
    def moveToPoint(self, x:float, y:float, theta:float):
        target_point = self.createGoal(x, y, theta)
        self.moveToGoal(target_point)
        
    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        quat_ = goal.target_pose.pose.orientation
        theta_ = tf.transformations.euler_from_quaternion((quat_.x, quat_.y, quat_.z, quat_.w))
        rospy.loginfo(f"Move to x: {goal.target_pose.pose.position.x} y: {goal.target_pose.pose.position.y} theta: {theta_[2]}")
        if success and state == GoalStatus.SUCCEEDED:
            print(" Complete")
            return True
        else:
            print(" Fail")
            self.move_base_action.cancel_goal()
            return False

def main():
    rospy.init_node('movetogoal_node')
    mba = moveBaseAction()
    
    while not rospy.is_shutdown():
        mba.moveToPoint(-0.64, 0.22, 0.0)
        rospy.sleep(1)

        mba.moveToPoint(-0.64, 0.22, 1.57)
        rospy.sleep(1)
        
        mba.moveToPoint(-0.64, 0.22, 3.14)
        rospy.sleep(1)
        
        mba.moveToPoint(-0.64, 0.22, -1.57)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass