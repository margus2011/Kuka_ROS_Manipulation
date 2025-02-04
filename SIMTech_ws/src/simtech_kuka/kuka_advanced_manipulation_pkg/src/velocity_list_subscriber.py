#!/usr/bin/env python
import rospy
from kuka_advanced_manipulation_pkg.msg import FloatList  # Import your custom message

def list_publisher():
    # Initialize the ROS Node
    rospy.init_node('velocity_list_publisher', anonymous=True)

    # Create a publisher object publishing to 'tcp_velocity' topic
    pub = rospy.Publisher('tcp_velocity', FloatList, queue_size=10)

    # Read data from file
    current_data = []
    with open('/home/jeeva/catkin_ws/src/GitHub/SIMTech_ws/src/industrial_robot_ros_packages/simtech_kuka/kuka_advanced_manipulation_pkg/src/data_text.txt', 'r') as file:
        for _ in range(5):  # Read five lines of data
            line = file.readline()
            if line:
                splitted = line.split()
                vel = float(splitted[4][1:])  # Assume the 5th element is the value after stripping the first character
                current_data.append(vel)
    
    # Initialize a counter for the number of publishes
    publish_count = 0

    # Continue running until the node is shut down or we publish twice
    while not rospy.is_shutdown() and publish_count < 2:
        # Log info about the list being published
        rospy.loginfo("Publishing float list data: %s", current_data)

        # Create and publish the message
        msg = FloatList()
        msg.data = current_data
        pub.publish(msg)
        publish_count += 1

        # Sleep for a second to ensure the message is sent
        rospy.sleep(1)
    
    # After publishing twice, shutdown the node
    rospy.signal_shutdown("Published twice, shutting down.")


if __name__ == '__main__':
    try:
        list_publisher()
    except rospy.ROSInterruptException:
        print("Node shutdown.")
