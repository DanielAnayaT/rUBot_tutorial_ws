#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

robot_x = 0
robot_y = 0

def pose_callback(pose):
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot X = %f\t Robot Y = %f\n",pose.x, pose.y)

def move_rubot(lin_velx,ang_vel,time_duration):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)

    vel = Twist()
    vel.linear.x = lin_velx
    vel.angular.z = ang_vel
    time_begin = rospy.Time.now()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        time_end = rospy.Time.now()
        duration = time_end - time_begin
        duration_s = duration.to_sec()

        if duration_s <= time_duration:
            rospy.loginfo("Robot running")
            pub.publish(vel)
        else:
            rospy.logwarn("Stopping robot")
            break
        
        rospy.loginfo("Time_end = " + str(time_end))
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('move_turtle', anonymous=False)#Has to be called here at the begining!
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        t= rospy.get_param("~t")
        move_rubot(v,w,t)
    except rospy.ROSInterruptException:
        pass