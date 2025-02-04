#!/usr/bin/env python
import rospy
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
import threading

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')
        self.subscriber = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.callback)
        self.publisher = rospy.Publisher('/trajectory_commands', JointTrajectoryPoint, queue_size=10)
        
    def callback(self, data):
        trajectory = data.trajectory[0]  # Assuming there's at least one trajectory in the message
        start_time = rospy.Time.now()
        
        for point in trajectory.joint_trajectory.points:
            delay = point.time_from_start.to_sec() - (rospy.Time.now() - start_time).to_sec()
            if delay > 0:
                rospy.sleep(delay)
            self.publish_point(point)
    
    def publish_point(self, point):
        # Here, you can filter out the accelerations or efforts if not needed
        point.effort = []
        point.accelerations = []
        self.publisher.publish(point)
        rospy.loginfo("Published joint trajectory point at time: %s", str(point.time_from_start))

if __name__ == '__main__':
    try:
        pf = PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
