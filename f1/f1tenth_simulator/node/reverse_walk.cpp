#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class ReverseWalker 
{
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double stable_speed;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;


public:
    ReverseWalker() 
    {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic;
        n.getParam("rev_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("stable_speed",stable_speed);
        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &ReverseWalker::odom_callback, this);
        
    }


    void odom_callback(const nav_msgs::Odometry & msg) {
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        double speed = stable_speed / 2 * -1;
        // set constant speed to be half of max speed
        drive_msg.speed = speed;
        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;
        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "reverse_walker");
    ReverseWalker re;
    ros::spin();
    return 0;
}