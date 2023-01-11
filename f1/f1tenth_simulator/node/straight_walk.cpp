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

class StraightWalker 
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
    StraightWalker() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic;
        n.getParam("straight_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("stable_speed",stable_speed);
        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &StraightWalker::odom_callback, this);
        
    }


    void odom_callback(const nav_msgs::Odometry & msg) {
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = stable_speed;
        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;
        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "straight_walker");
    StraightWalker sw;
    ros::spin();
    return 0;
}

/*
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

class StraightWalker 
{
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double stable_speed;

    // Listen for odom messages
    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;

    bool forward = true;;

//For avoidance detection
    double tta_threshold, ttr_threshold;
    
    //for vision narrowing 
    double min_los_p, max_los_p, min_los_n, max_los_n;
    //final holders
    double min_los, max_los;
    //For node management
    int currentNode = 0;
    //For delaying node behavior changes
    int counter = 0;
    int counter_max = 10800; 
    // To roughly keep track of vehicle state
    racecar_simulator::CarState state;
      // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;

public:
    StraightWalker() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic;
        n.getParam("straight_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("stable_speed",stable_speed);
        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
  // Initialize state
        state = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("width", width);
        n.getParam("wheelbase", wheelbase);
        n.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &StraightWalker::odom_callback, this);
        laser_sub = n.subscribe(scan_topic, 1, &StraightWalker::laser_callback, this);

 //Get params for avoidance detection 
        n.getParam("tta_threshold",tta_threshold);
                //Get params for reverse detection
        n.getParam("ttr_threshold", ttr_threshold);
        //Get params for vision narrowing
        //Front vision cone
        n.getParam("min_los_p",min_los_p);
        n.getParam("max_los_p",max_los_p);
        //Back vision cone
        n.getParam("min_los_n",min_los_n);
        n.getParam("max_los_n",max_los_n);

 // Precompute cosine and distance to car at each angle of the laser scan
        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, wheelbase, width, 
                scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);

    }

    void laser_callback(const sensor_msgs::LaserScan & msg) {
        los_checker();
        // check for a collision
        // collision_checker(msg);
        // reverse_checker(msg);
        // check for object to avoid
        avoidance_checker(msg);

    }
    bool turning;
    void avoidance_checker(const sensor_msgs::LaserScan & msg) 
    {
        
        // This function calculates ttr to see if there's a collision
        if (state.velocity != 0) 
        {
            for (size_t i = 0; i < msg.ranges.size(); i++) 
            {
                double angle = msg.angle_min + i * msg.angle_increment;
                // calculate projected velocity
                if (angle > min_los && angle < max_los)
                {
                    counter++;
                    double proj_velocity = state.velocity * cosines[i];
                    double tta = (msg.ranges[i] - car_distances[i]) / proj_velocity;
                    if(counter > counter_max)
                    {
                        counter -= counter_max;
                        // if it's smaller than 2 seconds, random!
                        if ((tta < tta_threshold) && (tta >= 0.0)) 
                        { 
                            
                            if (currentNode == 5)
                            {
                                currentNode = 3;
                                toggle_mux(random_walker_mux_idx, "Random Walker");
                            return;
                            }
                        }
                        // otherwise, keep walking straight.
                    else if( ttr > ttr_threshold*1.5 && currentNode == 3)
                        {
                            toggle_mux(straight_walker_mux_idx, "Straight Walker");
                            currentNode = 5;
                        }
                    }
                }
             }
       
        }
    }
 void los_checker()
    {
        if(forward )
            {
                min_los = min_los_p;
                max_los = max_los_p;
            }
        else
            {
                min_los = min_los_n;
                max_los = max_los_n;
            }
    }

    void odom_callback(const nav_msgs::Odometry & msg) {
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = stable_speed;
        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;
        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "straight_walker");
    StraightWalker sw;
    ros::spin();
    return 0;
}
*/