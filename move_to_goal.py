#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion


class Turtlebot:

    def __init__(self):

        rospy.init_node('turtlebot_controller', anonymous=True)

        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.goal_subscriber = rospy.Subscriber('/goal', Pose, self.update_goal)

        self.pose = Pose()

        self.yaw = 0

        self.goal_pose = Pose()

        self.goal_yaw = 0

        self.rate = rospy.Rate(10)

    
    def update_goal(self, data):

        self.goal_pose = data
        self.goal_pose.position.x = round(self.pose.position.x, 4)
        self.goal_pose.position.y = round(self.pose.position.y, 4)

        q = data.orientation
        q_list = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(q_list)
        self.goal_yaw = round(yaw, 4)

    
    def update_pose(self, data):

        self.pose = data.pose.pose
        self.pose.position.x = round(self.pose.position.x, 4)
        self.pose.position.y = round(self.pose.position.y, 4)

        q = data.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(q_list)
        self.yaw = round(yaw, 4)

    
    # Get distance to goal pose
    def euclidian_distance(self):

        return sqrt(pow((self.goal_pose.point.x - self.pose.point.x), 2) + pow((self.goal_pose.point.y - self.pose.point.y), 2))
    

    # Get angle difference between current pose and goal pose
    def angle_to_goal(self):

        return atan2(self.goal_pose.point.y - self.pose.point.y, self.goal_pose.point.x - self.pose.point.x)
    
    # Get angle difference to orient to final pose
    def diff_between_yaw(self):

        return (self.goal_yaw - self.yaw)
    

    # Determining velocities for our p controlller
    def linear_vel(self, constant=1):

        return (constant * self.euclidian_distance())
    

    def angular_vel(self, comp_angle, constant=1):

        return (constant * (comp_angle- self.yaw))


    def move_to_goal(self):

        goal = Pose()
        goal.position.x = 10
        goal.position.y = 10
        goal.position.z = 0
        goal.orientation.x = -0.03
        goal.orientation.y = -0.932
        goal.orientation.z = 0.311
        goal.orientation.w = 0.188
        
        self.update_goal(goal)

        DIST_TOLERANCE = 0.1
        ANGLE_TOLERANCE = 0.087

        vel_msg = Twist()

        # First circle. Orient towards goal
        while self.angle_to_goal() >= ANGLE_TOLERANCE:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(self.angle_to_goal())
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Second circle. Move forward
        while self.euclidian_distance() >= DIST_TOLERANCE:
            vel_msg.linear.x = self.linear_vel()
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

        # Third circle. Orient towards goal pose angle
        while self.angle_to_goal() >= ANGLE_TOLERANCE:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(self.diff_between_yaw())
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()


if __name__ == '__main__':

    try:
        x = Turtlebot()
        x.move_to_goal()
    except rospy.ROSInterruptException:
        pass
