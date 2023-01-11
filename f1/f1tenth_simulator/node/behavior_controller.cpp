#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <fstream>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <iostream>       // std::cout, std::endl
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>
#include <new>
using namespace racecar_simulator;

class BehaviorController {
private:
    // A ROS node
    ros::NodeHandle n;

    // Listen for messages from joystick, keyboard, laser scan, odometry, and IMU
    ros::Subscriber joy_sub;
    ros::Subscriber key_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber brake_bool_sub;
    ros::Subscriber temp_brake_bool_sub; 
    // Publisher for mux controller
    ros::Publisher mux_pub;
    ros::Publisher drive_pub;

    // Mux indices
    int joy_mux_idx;
    int key_mux_idx;
    int random_walker_mux_idx;
    int nav_mux_idx;
    int brake_mux_idx;
    int baa_mux_idx;
    int straight_walker_mux_idx;
    int reverse_walker_mux_idx;
    int reverse_random_walker_mux_idx;
    int hard_braker_mux_idx;
    int current_mux_idx;
    int evader_walker_mux_idx;
    // ***Add mux index for new planner here***
    // int new_mux_idx;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

    // Button indices
    int joy_button_idx;
    int key_button_idx;
    int random_walk_button_idx;
    int brake_button_idx;
    int nav_button_idx;
    int baa_button_idx;
    int straight_walk_button_idx;
    int reverse_walk_button_idx;
    // ***Add button index for new planner here***
    // int new_button_idx;

    // Key indices
    std::string joy_key_char;
    std::string keyboard_key_char;
    std::string brake_key_char;
    std::string random_walk_key_char;
    std::string nav_key_char;
    std::string baa_char;
    std::string straight_walk_key_char;
    std::string reverse_walk_key_char;
    std::string evader_walk_key_char;
    // ***Add key char for new planner here***
    // int new_key_char;

    // Is ebrake on? (not engaged, but on)
    bool safety_on;

    // To roughly keep track of vehicle state
    racecar_simulator::CarState state;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // for collision detection
    double ttc_threshold;
    bool in_collision=false;

    // for collision logging
    std::ofstream collision_file;
    double beginning_seconds;
    int collision_count=0;

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
    int counter_max = 1080; 
    bool braking;
    int nfCountL = 0;
    int nbCountL = 0;
    int wCountL = 0;
    int nfCountR = 0;
    int nbCountR = 0;
    int wCountR = 0;
    // int *nfIndex;
    // int *nbIndex;
    // int *wIndex;
public:
    BehaviorController() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string scan_topic, odom_topic, imu_topic, //drive_topic, 
        joy_topic, keyboard_topic, brake_bool_topic, mux_topic;
       // n.getParam("straight_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("imu_topic", imu_topic);
        n.getParam("joy_topic", joy_topic);
        n.getParam("mux_topic", mux_topic);
        n.getParam("keyboard_topic", keyboard_topic);
        n.getParam("brake_bool_topic", brake_bool_topic);

        // Make a publisher for mux messages
        mux_pub = n.advertise<std_msgs::Int32MultiArray>(mux_topic, 10);
       // drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start subscribers to listen to laser scan, joy, IMU, and odom messages
        laser_sub = n.subscribe(scan_topic, 1, &BehaviorController::laser_callback, this);
        joy_sub = n.subscribe(joy_topic, 1, &BehaviorController::joy_callback, this);
        imu_sub = n.subscribe(imu_topic, 1, &BehaviorController::imu_callback, this);
        odom_sub = n.subscribe(odom_topic, 1, &BehaviorController::odom_callback, this);
        key_sub = n.subscribe(keyboard_topic, 1, &BehaviorController::key_callback, this);
        brake_bool_sub = n.subscribe(brake_bool_topic, 1, &BehaviorController::brake_callback, this);
        // Get mux indices
        n.getParam("joy_mux_idx", joy_mux_idx);
        n.getParam("key_mux_idx", key_mux_idx);
        n.getParam("random_walker_mux_idx", random_walker_mux_idx);
        n.getParam("brake_mux_idx", brake_mux_idx);
        n.getParam("nav_mux_idx", nav_mux_idx);
        n.getParam("baa_mux_idx",baa_mux_idx);
        n.getParam("straight_walker_mux_idx", straight_walker_mux_idx);
        n.getParam("reverse_walker_mux_idx", reverse_walker_mux_idx);
        n.getParam("reverse_random_walker_mux_idx",reverse_random_walker_mux_idx);
        n.getParam("hard_braker_mux_idx",hard_braker_mux_idx);
        n.getParam("evader_walker_mux_idx",evader_walker_mux_idx);
        // ***Add mux index for new planner here***
        // n.getParam("new_mux_idx", new_mux_idx);

        // Get button indices
        n.getParam("joy_button_idx", joy_button_idx);
        n.getParam("key_button_idx", key_button_idx);
        n.getParam("random_walk_button_idx", random_walk_button_idx);
        n.getParam("brake_button_idx", brake_button_idx);
        n.getParam("nav_button_idx", nav_button_idx);
        n.getParam("baa_button_idx",baa_button_idx);
        n.getParam("straight_walk_button_idx", straight_walk_button_idx);
        n.getParam("reverse_walk_button_idx", reverse_walk_button_idx);
        // ***Add button index for new planner here***
        // n.getParam("new_button_idx", new_button_idx);

        // Get key indices
        n.getParam("joy_key_char", joy_key_char);
        n.getParam("keyboard_key_char", keyboard_key_char);
        n.getParam("random_walk_key_char", random_walk_key_char);
        n.getParam("brake_key_char", brake_key_char);
        n.getParam("nav_key_char", nav_key_char);
        n.getParam("baa_char",baa_char);
        n.getParam("straight_walk_key_char",straight_walk_key_char);
        n.getParam("reverse_walk_key_char",reverse_walk_key_char);
        n.getParam("evader_walk_key_char",evader_walk_key_char);
        // ***Add key char for new planner here***
        // n.getParam("new_key_char", new_key_char);

        // Initialize the mux controller 
        n.getParam("mux_size", mux_size);
        mux_controller.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }

        // Start with ebrake off
        safety_on = false;

        // Initialize state
        state = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};

        // Get params for precomputation and collision detection
        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        n.getParam("ttc_threshold", ttc_threshold);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("width", width);
        n.getParam("wheelbase", wheelbase);
        n.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;

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

        // Create collision file to be written to
        std::string filename;
        n.getParam("collision_file", filename);
        collision_file.open(ros::package::getPath("f1tenth_simulator") + "/logs/" + filename + ".txt");
        beginning_seconds = ros::Time::now().toSec();
    }

    /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    void publish_mux() {
        // make mux message
        std_msgs::Int32MultiArray mux_msg;
        mux_msg.data.clear();
        // push data onto message
        for (int i = 0; i < mux_size; i++) {
            mux_msg.data.push_back(int(mux_controller[i]));
        }

        // publish mux message
        mux_pub.publish(mux_msg);
    }

    void change_controller(int controller_idx) 
    {
        // This changes the controller to the input index and publishes it

        // turn everything off
        for (int i = 0; i < mux_size; i++) 
        {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[controller_idx] = true;

        publish_mux();
    }
    double LOSIncrement = 0.044004;
    int *narrowFrontLeft;
    int *narrowBackLeft;
    int *wideLeft;
    int *narrowFrontRight;
    int *narrowBackRight;
    int *wideRight;
    
    int *area;
    void scanForCollidables(const sensor_msgs::LaserScan & msg)
    {
        //Minimum angle increment 0.00582316.
        //Minimum angle is -3.14159
        //From this we can determine
        //Range: -0.044004 and 0.044004  
        //         index 532 -> 547 a size of 16
        //Range: -0.44004 and 0.44004
        //index 464 -> 531 and 548 -> 616 a size of 136
        //I used a matlab function i made, "Scan angle indexer"
        narrowFrontLeft = new int[8];
        narrowBackLeft = new int[16];
        wideLeft = new int[68];
        narrowFrontRight = new int[8];
        narrowBackRight = new int[16];
        wideRight = new int[68];
        int messages[1080];
        
        int nfIndexLeft[8] = 
        {533,534,535,536,537,538,539,540};
        //nfIndex = nfIndex1;
        int nbIndexLeft[16] =
        {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
        
        //nbIndex = nbIndex1;
        int wIndexLeft[68] =
        {
            465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,
            485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,
            505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,
            525,526,527,528,529,530,531,532
        };
        int nfIndexRight[8] = 
        {541,542,543,544,545,546,547,549};
        //nfIndex = nfIndex1;
        int nbIndexRight[16] =
        {1065,1066,1067,1068,1069,1070,1071,1072,1073,1074,1075,1076,1077,1078,1079,1080};
        
        //nbIndex = nbIndex1;
        int wIndexRight[68] =
        {
            549,550,551,552,553,554,555,556,557,558,559,560,
            561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,
            581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,599,600,
            601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616
        };


        //wIndex = wIndex1;
        // indexCheckForDistance(tta_threshold, nfIndex, 
        // narrowFront, msg);
        // indexCheckForDistance(tta_threshold, nbIndex, 
        // narrowBack, msg);
        // indexCheckForDistance(tta_threshold, wIndex, 
        // wide, msg);

        nfCountL = 0;
        for(int i : nfIndexLeft)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold)
            {
                narrowFrontLeft[nfCountL] = time_to_intercept;
                nfCountL++;
            }
        } 
        nfCountR = 0;
        for(int i : nfIndexRight)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold)
            {
                narrowFrontRight[nfCountR] = time_to_intercept;
                nfCountR++;
            }
        } 
        nbCountL = 0;
        for(int i : nbIndexLeft)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold)
            {
                narrowBackLeft[nbCountL] = time_to_intercept;
               nbCountL++;
            }
        }
        nbCountR = 0;
        for(int i : nbIndexRight)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold)
            {
                narrowBackRight[nbCountR] = time_to_intercept;
               nbCountR++;
            }
        }
        wCountL = 0;
        for(int i : wIndexLeft)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold)
            {
                wideLeft[wCountL] = time_to_intercept;
                wCountL++;
            }
        }
        wCountR = 0;
        for(int i : wIndexRight)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold)
            {
                wideRight[wCountR] = time_to_intercept;
                wCountR++;
            }
        }
        
    }
    // int indexCheckForDistance(double thresholdVal, int index_Array [], 
    // int * array_of_close_points,const sensor_msgs::LaserScan & msg)
    // {
    //     int w = 0;
    //     double angle;
    //     int arr [] = index_Array;
    //     for(int i : index_Array)
    //     {
    //         angle = msg.angle_min + i*msg.angle_increment;
    //         double proj_velocity = state.velocity * cosines[i];
    //         double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
    //         if(time_to_intercept <= thresholdVal)
    //         {
    //             array_of_close_points[w] = time_to_intercept;
    //             w++;
    //         }
    //     }
    //     return array_of_close_points
    // }
    bool distantProximityCheck(const sensor_msgs::LaserScan & msg, 
    bool front, bool back)
    {
        
        los_checker();
        double angle;
        
    }

    void avoidance_checker(const sensor_msgs::LaserScan & msg) 
    {
        
        // This function calculates tta to see if there's a collision
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
                                
                                for(int i  = 0; i < 2000; i++)
                                {
                                    temporary_toggle_brake_mux(random_walker_mux_idx);
                                    mux_on(random_walker_mux_idx, "Random Walker");
                                return;
                                }
                            }
                            // otherwise, keep walking straight.
                        else if( tta > tta_threshold*1.5 && currentNode == 3)
                            {
                                temporary_toggle_brake_mux(random_walker_mux_idx);
                                mux_on(straight_walker_mux_idx, "Straight Walker");
                                currentNode = 5;
                            }
                        }   
                    }
                }
       
            }
        }
    }
    int frontClear = 0;
    int backClear = 0;
    double angle;
    void reverse_checker(const sensor_msgs::LaserScan & msg)
    {
        if(currentNode != 6)
        {
            for (size_t i = 0; i < msg.ranges.size(); i++) 
            {
                angle = msg.angle_min + i * msg.angle_increment;
                if (angle >= min_los && angle <= max_los)
                {
                        counter++;
                    // calculate projected velocity
                    double proj_velocity = state.velocity * cosines[i];
                    double ttr = (msg.ranges[i] - car_distances[i]) / proj_velocity;
                    if(counter > counter_max)
                    {
                            counter-=counter_max;
                        // if it's smaller than 3 seconds, avoid it!
                        if ((ttr < ttr_threshold) && (ttr >= 0.0)) 
                        { 
                        //Reverse!
                            currentNode = 6;
                            temporary_toggle_brake_mux(reverse_walker_mux_idx);
                            //toggle_mux(reverse_walker_mux_idx, "Reverse Walker");
                        }
                    }
                }
            }
        }
        else 
        {
             for (size_t i = 0; i < msg.ranges.size(); i++) 
             {
                angle = msg.angle_min + i * msg.angle_increment;
                //Check in front
                if (angle >= min_los_p && angle <= max_los_p)
                {
                        counter++;
                    // calculate projected velocity
                    double proj_velocity = state.velocity * cosines[i];
                    double ttr = (msg.ranges[i] - car_distances[i]) / proj_velocity;
                        
                    if(counter > counter_max)
                    {
                        counter-=counter_max;
                        // if it's smaller than 3 seconds, avoid it!
                        if ((ttr < ttr_threshold) && (ttr >= 0.0)) 
                        { 
                            //The way is no good, Move Randomly!
                            currentNode = 3;
                            temporary_toggle_brake_mux(reverse_walker_mux_idx);
                            frontClear = 1;
                            goto BEHIND;
                            
                        }
                        else if ((ttr > ttr_threshold*1.5))
                        {
                            //The way is clear, go forward.
                            currentNode = 5;
                            temporary_toggle_brake_mux(straight_walker_mux_idx);
                            frontClear = 2;
                        }
                    }
                }
             }
             BEHIND: 
             for (size_t i = 0; i < msg.ranges.size(); i++) 
             {
                
                //check behind
                if (angle >= min_los_n && angle <= max_los_n)
                {
                        counter++;
                    // calculate projected velocity
                    double proj_velocity = state.velocity * cosines[i];
                    double ttr = (msg.ranges[i] - car_distances[i]) / proj_velocity;
                        
                    if(counter > counter_max)
                    {
                            counter-=counter_max;
                        // if it's smaller than 3 seconds, avoid it!
                        if ((ttr < ttr_threshold) && (ttr >= 0.0)) 
                        { 
                        //Walk random
                            currentNode = 3;
                            temporary_toggle_brake_mux(random_walker_mux_idx);
                            backClear = 1;
                            

                        }
                        // else if(ttr > ttr_threshold*1.5)
                        // {
                            
                        // }
                    }
                }
            }
        }
    }
    void collision_checker(const sensor_msgs::LaserScan & msg) {
        // This function calculates TTC to see if there's a collision
        // ROS_INFO_STREAM("angle min: ");
        // ROS_INFO_STREAM(msg.angle_min);
        // ROS_INFO_STREAM("angle increment: ");
        // ROS_INFO_STREAM(msg.angle_increment);
        
        if (state.velocity != 0) {
            for (size_t i = 0; i < msg.ranges.size(); i++) {
                angle = msg.angle_min + i * msg.angle_increment;

                // calculate projected velocity
                double proj_velocity = state.velocity * cosines[i];
                double ttc = (msg.ranges[i] - car_distances[i]) / proj_velocity;

                // if it's small, there's a collision
                if ((ttc < ttc_threshold) && (ttc >= 0.0)) { 
                    // Send a blank mux and write to file
                    collision_helper();

                    in_collision = true;

                    collision_count++;
                    collision_file << "Collision #" << collision_count << " detected:\n";
                    collision_file << "TTC: " << ttc << " seconds\n";
                    collision_file << "Angle to obstacle: " << angle << " radians\n";
                    collision_file << "Time since start of sim: " << (ros::Time::now().toSec() - beginning_seconds) << " seconds\n";
                    collision_file << "\n";
                    return;
                }
            }
            // if it's gone through all beams without detecting a collision, reset in_collision
            in_collision = false;
        }
    }

    void collision_helper() {
        // This function will turn off ebrake, clear the mux and publish it

        safety_on = false;

        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }

        publish_mux();
    }
    
    void mux_on(int mux_idx, std::string driver_name)
    {
        if (mux_controller[mux_idx]) {
            return;
        }
        else {
            for (int i = 0; i < mux_size; i++) 
            {
            mux_controller[i] = false;
            }
            ROS_INFO_STREAM(driver_name << " turned on");
            current_mux_idx = mux_idx;
            change_controller(mux_idx);
        }
    }
    void mux_off(int mux_idx, std::string driver_name)
    {
        if (mux_controller[mux_idx]) {
            ROS_INFO_STREAM(driver_name << " turned off");
            mux_controller[mux_idx] = false;
            publish_mux();        }
        else {
            return;
        }
    }
    void toggle_mux(int mux_idx, std::string driver_name) {
        // This takes in an index and the name of the planner/driver and 
        // toggles the mux appropiately
        if (mux_controller[mux_idx]) {
            ROS_INFO_STREAM(driver_name << " turned off");
            mux_controller[mux_idx] = false;
            publish_mux();
        }
        else {
            ROS_INFO_STREAM(driver_name << " turned on");
            current_mux_idx = mux_idx;
            change_controller(mux_idx);
        }
    }

    void toggle_brake_mux() {
        ROS_INFO_STREAM("Emergency brake engaged");
        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[brake_mux_idx] = true;

        publish_mux();
    }

     void temporary_toggle_brake_mux(int prior_mux_idx) {
        ROS_INFO_STREAM("brake engaged");
        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[hard_braker_mux_idx] = true;
        //&BehaviorController::brake_hard;
        current_mux_idx = hard_braker_mux_idx;
        // ROS_INFO_STREAM("Start wait");
        // std::this_thread::sleep_for (std::chrono::seconds(1));
        // ROS_INFO_STREAM("End wait");
                 braking =true;
                 destination_mux = prior_mux_idx;
                 publish_mux();
    }
    int destination_mux;
    /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void brake_callback(const std_msgs::Bool & msg) {
        if (msg.data && safety_on) {
            toggle_brake_mux();
        } else if (!msg.data && mux_controller[brake_mux_idx]) {
            mux_controller[brake_mux_idx] = false;
        }
    }
    void temporary_brake_callback(const std_msgs::Bool & msg, int previous_mux_idx) {
        if (msg.data && safety_on) {
            temporary_toggle_brake_mux(previous_mux_idx);
        } else if (!msg.data && mux_controller[brake_mux_idx]) {
            mux_controller[brake_mux_idx] = false;
        }
    }
    void joy_callback(const sensor_msgs::Joy & msg) {
        // Changing mux_controller:
        if (msg.buttons[joy_button_idx]) { 
            // joystick
            toggle_mux(joy_mux_idx, "Joystick");
        }
        if (msg.buttons[key_button_idx]) { 
            // keyboard
            toggle_mux(key_mux_idx, "Keyboard");
        }
        else if (msg.buttons[brake_button_idx]) { 
            // emergency brake 
            if (safety_on) {
                ROS_INFO("Emergency brake turned off");
                safety_on = false;
            }
            else {
                ROS_INFO("Emergency brake turned on");
                safety_on = true;
            }
        }
        else if (msg.buttons[random_walk_button_idx]) 
        { 
            // random walker
            currentNode = 3;
            toggle_mux(random_walker_mux_idx, "Random Walker");
        } 
        else if (msg.buttons[nav_button_idx]) 
        {
            // nav
            toggle_mux(nav_mux_idx, "Navigation");
        }
        else if (msg.buttons[baa_button_idx])
        {
            //basic area awareness
            //Debug.log("So i uh... don't seem to be working.");
            toggle_mux(baa_mux_idx,"basic_area_awareness");
        }
        else if (msg.buttons[straight_walk_button_idx]) 
        { 
            // random walker
            toggle_mux(straight_walker_mux_idx, "Straight Walker");
        } 
        else if (msg.buttons[reverse_walk_button_idx]) 
        { 
            // random walker
            currentNode = 6;
            toggle_mux(reverse_walker_mux_idx, "Reverse Walker");
        } 
        // ***Add new else if statement here for new planning method***
        // if (msg.buttons[new_button_idx]) {
        //  // new planner
        //  toggle_mux(new_mux_idx, "New Planner");
        // }

    }

    void key_callback(const std_msgs::String & msg) {
        // Changing mux controller:
        if (msg.data == joy_key_char) {
            // joystick
            toggle_mux(joy_mux_idx, "Joystick");
        } else if (msg.data == keyboard_key_char) {
            // keyboard
            toggle_mux(key_mux_idx, "Keyboard");
        } else if (msg.data == brake_key_char) {
            // emergency brake 
            if (safety_on) {
                ROS_INFO("Emergency brake turned off");
                safety_on = false;
            }
            else {
                ROS_INFO("Emergency brake turned on");
                safety_on = true;
            }
        } 
        else if (msg.data == random_walk_key_char) 
        {
            // random walker
            //Debug.log("random hit.");
		ROS_INFO("Random World");
        
		//std::cout << "Rando hit" << endl;
        currentNode = 3;
            toggle_mux(random_walker_mux_idx, "Random Walker");
        } 
        else if (msg.data == nav_key_char) {
            // nav
            toggle_mux(nav_mux_idx, "Navigation");
        }
        else if(msg.data == baa_char)
        {
		//ROS_INFO("Hello World");
            //Debug.log("So i uh... don't seem to be working.");
            toggle_mux(baa_mux_idx,"basic_area_awareness");
        }
        else if (msg.data == straight_walk_key_char) 
        {
            // random walker
            //Debug.log("random hit.");
            currentNode = 5;
		ROS_INFO("Straight World");
        
		//std::cout << "Rando hit" << endl;
            toggle_mux(straight_walker_mux_idx, "Straight Walker");
        } 
         else if (msg.data == reverse_walk_key_char) 
        {
            // random walker
            //Debug.log("random hit.");
            currentNode = 6;
		ROS_INFO_STREAM("Reverse World");
        
		//std::cout << "Rando hit" << endl;
            toggle_mux(reverse_walker_mux_idx, "Reverse Walker");
        } 
        else if (msg.data == evader_walk_key_char) 
        {
            // Evader walker
		ROS_INFO_STREAM("Evader Activated");
        
            toggle_mux(evader_walker_mux_idx, "Evader Walker");
        } 
        // ***Add new else if statement here for new planning method***
        // if (msg.data == new_key_char) {
        //  // new planner
        //  toggle_mux(new_mux_idx, "New Planner");
        // }

    }

    void laser_callback(const sensor_msgs::LaserScan & msg) {
        // scanForCollidables(msg);
        // avoidanceLogic();
        // if(!braking)
        // { 
        // los_checker();
        // // check for a collision
        // collision_checker(msg);
        // reverse_checker(msg);
        // // check for object to avoid
        // avoidance_checker(msg);
        // }
    }
    int brakeCounter = 0;
    int brakeMax = 50;
    int waitToLogicCounter = 0;
    int logicMax = 0;
    void avoidanceLogic()
    {
        int minNBvalL = 100;
        if(nbCountL > 0)
        {
            for(int i = 0; i < nbCountL; i++)
            {
                if(narrowBackLeft[i] < minNBvalL && narrowBackLeft[i] > 0)
                minNBvalL = narrowBackLeft[i];
            }
        }
        int minNBvalR = 100;
        if(nbCountR > 0)
        {
            for(int i = 0; i < nbCountR; i++)
            {
                if(narrowBackRight[i] < minNBvalR && narrowBackRight[i] > 0)
                minNBvalL = narrowBackRight[i];
            }
        }
        int minNFvalL = 100;
        if(nfCountL > 0)
        {
            for(int i = 0; i < nfCountL; i++)
            {
                if(narrowFrontLeft[i] < minNFvalL && narrowFrontLeft[i] > 0)
                minNFvalL = narrowFrontLeft[i];
            }
        }
        int minNFvalR = 100;
        if(nfCountR > 0)
        {
            for(int i = 0; i < nfCountR; i++)
            {
                if(narrowFrontRight[i] < minNFvalR && narrowFrontRight[i] > 0)
                minNFvalR = narrowFrontRight[i];
            }
        }
        int minWvalL = 100;
        if(wCountL > 0)
        {
            for(int i = 0; i < wCountL; i++)
            {
                if(wideLeft[i] < minWvalL && wideLeft[i] > 0)
                minWvalL  = wideLeft[i];
            }
        }
        int minWvalR = 100;
        if(wCountR > 0)
        {
            for(int i = 0; i < wCountR; i++)
            {
                if(wideRight[i] < minWvalR && wideRight[i] > 0)
                minWvalR  = wideRight[i];
            }
        }

        int minNFval =10;
        int minNBval = 10;
        int minWval = 10;
        if(minNFvalR <= minNFvalL)
        minNFval = minNFvalR;
        else
        minNFval = minNFvalL;

        if(minNBvalL <= minNFvalR)
        minNBval = minNBvalL;
        else
        minNBval = minNBvalR;

        if(minWvalL <= minWvalR)
        minWval = minWvalL;
        else
        minWval = minWvalR;
        // ROS_INFO_STREAM("NB count is: ");
        // ROS_INFO_STREAM(nbCountL);
        // ROS_INFO_STREAM("min_NB: ");
        // ROS_INFO_STREAM(minNBval);
        
        switch(current_mux_idx)
        {
            case 2: //random walker
            //If something is close to the narrowfront,
                //check wider angle to see if anything on either side 
                //check reverse to see if anything behind car. 
                    //if wider angle sees stuff and reverse is clear
                    // just reverse straight back
                    //if wide angle doesn't see anything and reverse is clear
                    // use reverse random
                    //if wide angle and reverse aren't clear brake 
                        //try again after a few seconds 
            //If nothing is in narrow front, go straight. 
            waitToLogicCounter++;
            if(waitToLogicCounter < logicMax)
            {
                 waitToLogicCounter -= logicMax;
                if(minNFval  < 2)
                {
                    if(minWval < 1)
                    {
                        if(minNBval < 1)
                        {
                        //temporary_toggle_brake_mux(random_walker_mux_idx);
                            //brake and try again
                        temporary_toggle_brake_mux(reverse_walker_mux_idx); 
                        ROS_INFO_STREAM ("closing in: ");
                        ROS_INFO_STREAM (minNBval);

                        }
                        else
                        {
                            temporary_toggle_brake_mux(reverse_walker_mux_idx);
                            ROS_INFO_STREAM ("only back open");
                            //reverse backwards.
                        }
                    }
                    else 
                    {  

                        if (minNBval >1)
                        {
                            temporary_toggle_brake_mux(reverse_random_walker_mux_idx);
                            ROS_INFO_STREAM ("back and side wide open");
                            //reverse random
                        }
                        else
                        {
                            temporary_toggle_brake_mux(reverse_walker_mux_idx); 
                            ROS_INFO_STREAM ("only side wide open");
                        }
                        
                    }
                }
                else
                {
                    ROS_INFO_STREAM ("just go straight");
                    //straight
                    temporary_toggle_brake_mux(straight_walker_mux_idx);
                }
            }
            break;
            case 6: //straight
                //if something is in the narrow front then brake 
                    //Check if the sides are clear
                        //if sides aren't clear 
                            //check if back clear
                                //if back clear, 
                                    //select reverse straight after brake    
                        //if sides are clear 
                            //select random walker after brake
                            waitToLogicCounter++;
                if(waitToLogicCounter > logicMax)
                {
                     waitToLogicCounter -= logicMax;
                    if(minNFval  < 2)
                    {
                        if(minWval < 1)
                        {
                            if(minNBval < 2)
                            {
                                ROS_INFO_STREAM("Oh shit straight trapped me");
                                //oh shit. i'm trapped. 
                            }
                            else
                            {
                                ROS_INFO_STREAM("braking to reverse");
                                temporary_toggle_brake_mux(reverse_walker_mux_idx);
                                //brake and reverse straight
                            }
                        }
                        else 
                        {  
                            ROS_INFO_STREAM("Random walker activate");
                            temporary_toggle_brake_mux(random_walker_mux_idx);
                            //brake and random walk
                            
                        }
                    }
                }
            break;
            case 7: //reverse straight
            //if something is in the narrow back then brake 
                    //Check if the sides are clear
                        //if sides aren't clear 
                            //check if narrow front clear
                                //if narrow front clear, 
                                    //select straight after brake    
                        //if sides are clear 
                            //check if narrow front clear 
                                //if narrow front clear 
                                    //select random walker after brake
                                //if narrow front isn't clear 
                                    //select... i need a better logic for this
                waitToLogicCounter++;
            if(waitToLogicCounter > logicMax)
            {
                 waitToLogicCounter -= logicMax;
                if(minNBval  < 2)
                {
                    if(minWval < 2)
                    {
                        if(minNFval < 2)
                        {
                            //brake and try again
                            temporary_toggle_brake_mux(reverse_walker_mux_idx);
                        }
                        else
                        {
                            //brake and go straight
                            temporary_toggle_brake_mux(straight_walker_mux_idx);
                        }
                    }
                    else 
                    {  
                        
                        if (minNFval < 2)
                        {
                            // i need better logic 
                        }
                        else
                        {
                            //brake and random walk
                            temporary_toggle_brake_mux(random_walker_mux_idx);
                        } 
                        
                    }
                }
                else
                {
                    //nothing
                } 
            }
            break;
            case 8: //reverse random
            //If something is close to the narrowback,
                //check wider angle to see if anything on either side 
                //check front to see if anything in front of car. 
                    //if wider angle sees stuff and front is clear
                        // just go straight forward after brake
                    //if wide angle doesn't see anything and straight is clear
                        // use random walker after brake
                    //if wide angle and front aren't clear brake 
                        //try again after a few seconds 
            //If nothing is in narrow front, go straight. 
            waitToLogicCounter++;
            if(waitToLogicCounter > logicMax)
            {
                waitToLogicCounter -= logicMax;
                if(minNBval  < 2)
                {
                    if(minWval < 2)
                    {
                        if(minNFval < 2)
                        {
                            temporary_toggle_brake_mux(reverse_random_walker_mux_idx);
                            //brake and try again
                        }
                        else
                        {
                            //brake and go straight
                            temporary_toggle_brake_mux(straight_walker_mux_idx);
                        }
                    }
                    else 
                    {  
                        
                        if (minNFval < 2)
                        {
                            // i need better logic 
                        }
                        else
                        {
                            //brake and random walk
                            temporary_toggle_brake_mux(random_walker_mux_idx);
                        } 
                        
                    }
                }
                else
                {
                    temporary_toggle_brake_mux(straight_walker_mux_idx);
                    //Go straight
                } 
            }
            break;  
            case 9://brake
                //if braking, brake counter increases each time it's checked
                //after enough checks it resumes whatever it was doing. '
            //counter ++
                //if counter > brake counter max
                //resume whatever thing i checked for next
                brakeCounter++;
                ROS_INFO_STREAM("BRAKING LIKE A BITCH");
                if(brakeCounter > brakeMax)
                {
                    ROS_INFO_STREAM("BRAKE OFF");
                    brakeCounter -= brakeMax;
                    mux_on(destination_mux, "prior mux on");
                }
            break;
            default:
               // ROS_INFO_STREAM("default checker entered");
            break;
        }
    }
    void los_checker()
    {
        if(currentNode != 6 )
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
    // void brake_hard(const nav_msgs::Odometry & msg)
    // {
    //     ackermann_msgs::AckermannDriveStamped drive_st_msg;
    //     ackermann_msgs::AckermannDrive drive_msg;
    //     ROS_INFO_STREAM("BRAKING HARD");
    //     drive_msg.speed = 0.0;
    //     drive_st_msg.drive = drive_msg;
    //     drive_pub.publish(drive_st_msg);

    // }
    void odom_callback(const nav_msgs::Odometry & msg) {
        // Keep track of state to be used elsewhere
        state.velocity = msg.twist.twist.linear.x;
        state.angular_velocity = msg.twist.twist.angular.z;
        state.x = msg.pose.pose.position.x;
        state.y = msg.pose.pose.position.y;
        // if(braking && state.velocity  < 0.01 && state.velocity > 0.01)
        // {
        //     // ROS_INFO_STREAM("SLEEPING");
        //     // std::this_thread::sleep_for(std::chrono::seconds(1));
        //     // ROS_INFO_STREAM("AWAKE");
        //     braking = false;
        //     toggle_mux(prior_mux, "brake off, prior ");
        // }
    }

    void imu_callback(const sensor_msgs::Imu & msg) {

    }


};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "behavior_controller");
    BehaviorController bc;
    ros::spin();
    return 0;
}
