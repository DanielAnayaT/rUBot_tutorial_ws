#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'move_turtle' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('move_turtle', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.goal_pose = Pose()
        self.goal_pose.x = rospy.get_param("~x")
        self.goal_pose.y = rospy.get_param("~y")
        self.distance_tolerance = rospy.get_param("~tol")
        self.theta_target = rospy.get_param("~theta")
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = self.pose.theta

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = self.goal_pose.x
        goal_pose.y = self.goal_pose.y
        distance_tolerance = self.distance_tolerance
        theta_target = self.theta_target
        
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Position reached! Adjusting orientation...")
        
        while abs(theta_target - self.pose.theta) > 0.05:
            vel_msg.angular.z = 2.0 * (theta_target - self.pose.theta)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Orientation reached! Stopping...")

if __name__ == '__main__':
    try:
        turtle = TurtleBot()
        turtle.move2goal()
    except rospy.ROSInterruptException:
        pass