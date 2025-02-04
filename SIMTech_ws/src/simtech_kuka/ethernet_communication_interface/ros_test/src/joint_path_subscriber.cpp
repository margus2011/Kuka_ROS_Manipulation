#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <vector>
#include "kuka_advanced_manipulation_pkg/FloatList.h" 

std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points;
size_t current_point_index = 0;

void velocityCallback(const kuka_advanced_manipulation_pkg::FloatList::ConstPtr& msg)
{
    ROS_INFO("Received velocity list:");
    for (const float& v : msg->data) {
        ROS_INFO("%f", v);
    }
}

void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
         static int message_count = 0; // Counter for messages received

    // Increment message counter
    message_count++;

    // Process only from the second message onwards
     if (message_count > 1) {
        // Clear previous trajectory points
        trajectory_points.clear();

        // Store new trajectory points
        trajectory_points = msg->points;

        current_point_index = 0;  // Reset the index whenever a new message is received

        // Print each trajectory point
        /*for (size_t i = 0; i < trajectory_points.size(); i++) {
            const auto& point = trajectory_points[i];
            std::string positions, velocities;

            // Collect all positions for this point
            for (auto pos : point.positions) {
                positions += std::to_string(pos) + " ";
            }

            // Collect all velocities for this point
            for (auto vel : point.velocities) {
                velocities += std::to_string(vel) + " ";
            }

            // Log positions and velocities
            ROS_INFO("Point %ld: Positions: [%s]", i+1, positions.c_str());
            //ROS_INFO("Point %ld: Velocities: [%s]", i+1, velocities.c_str());
        }*/
    }
}

void processTrajectoryPoint(const ros::TimerEvent&) {
    if (current_point_index < trajectory_points.size()) {
        const auto& point = trajectory_points[current_point_index];
        ROS_INFO("Processing trajectory point %lu", current_point_index + 1);
        for (size_t i = 0; i < point.positions.size(); ++i) {
            ROS_INFO("Joint %ld Position: %f", i, point.positions[i]);
        }
        current_point_index++;  // Move to the next point
    } else {
        ROS_INFO("All trajectory points have been processed.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_path_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("tcp_velocity", 1000, velocityCallback);

    //ros::Subscriber sub = nh.subscribe("joint_path_command", 10, trajectoryCallback);
    
    /*ros::Subscriber sub = nh.subscribe("joint_path_command", 10, 
        [](const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
            trajectory_points = msg->points;
            current_point_index = 0;  // Reset the index whenever a new message is received
            ROS_INFO("Received new trajectory with %ld points.", trajectory_points.size());
        }
    );*/
    
    //Set up a timer to process each point with a delay
    //ros::Timer timer = nh.createTimer(ros::Duration(1.0), processTrajectoryPoint, false);


    ros::spin();

    return 0;
}