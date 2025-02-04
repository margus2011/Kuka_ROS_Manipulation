#!/usr/bin/env python
import rospy
from kuka_advanced_manipulation_pkg.msg import Velocity  # Import your custom message

def velocity_publisher():
    # Initialize the ROS Node
    rospy.init_node('velocity_publisher', anonymous=True)

    # Create a publisher object publishing to 'tcp_velocity' topic
    pub = rospy.Publisher('tcp_velocity', Velocity, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize the last published message
    last_published_msg = Velocity()

    # Keep publishing until the node is shut down
    while not rospy.is_shutdown():
        # Create a new Velocity message
        vel_msg = Velocity()

        # Set linear and angular velocities
        vel_msg.tcp_velocity = 1.0  # Linear velocity in m/s
        vel_msg.point_index = 0.5  # Angular velocity in rad/s

        # Check if the current message is different from the last published message
        if (vel_msg.tcp_velocity != last_published_msg.tcp_velocity or
            vel_msg.point_index != last_published_msg.point_index):
            
            # Log info about the velocity being published
            rospy.loginfo("Publishing new velocity: Linear Velocity: %s, Angular Velocity: %s",
                          vel_msg.tcp_velocity, vel_msg.point_index)

            # Publish the message
            pub.publish(vel_msg)

            # Update the last published message
            last_published_msg = vel_msg

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
