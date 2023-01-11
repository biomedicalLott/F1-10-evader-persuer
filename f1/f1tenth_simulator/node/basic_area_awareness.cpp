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

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// for printing
#include <iostream>
#include <thread>         // //std::this_thread::sleep_for
#include <chrono>         // std::chrono::milliseconds
// for RAND_MAX
#include <cstdlib>

using namespace racecar_simulator;
using namespace std;
class basic_area_awareness
{
    private:
    
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double baa_speed;
    double max_steering_angle;

    //Listen for laser messages 
    ros::Subscriber laser_sub;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // Publisher for mux controller
  //  ros::Publisher mux_pub;

    // previous desired steering angle
    double prev_angle=0.0;
    bool avoiding_obstacle = false;
    double tta_threshold;
    double ttr_threshold;

        // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;
  //for vision narrowing 
    double min_los_p, max_los_p, min_los_n, max_los_n;
    //final holders
    double min_los, max_los;
    //For node management
    int currentNode = 0;
    //For delaying node behavior changes
     int counter = 0;
    int counter_max = 1080; 
  // To roughly keep track of vehicle state
    racecar_simulator::CarState state;
    

    bool braking;
    //Values for scanning for collidables//
    //These are counts of values that are within a reasonable distance 
    //for  narrow front, back,and wide fov for left and right sides 
    int nfCountL = 0;
    int nbCountL = 0;
    int wCountL = 0;
    int nfCountR = 0;
    int nbCountR = 0;
    int wCountR = 0;
    //line of site increment in radians 
        double LOSIncrement = 0.044004;
    //Storage for the values found under the threshold
    int narrowFrontLeft[15];
    int narrowBackLeft[16];
    int wideLeft[76];
    int narrowFrontRight[15];
    int narrowBackRight[16];
    int wideRight[76];
    

    int area[1080];
    //minimums within the arrays of values under the thresholds. 
    int minNFvalL = 0;
    int minNFvalR = 0;
    int minNBvalL = 0;
    int minNBvalR = 0;
    int minWvalL = 0;
    int minWvalR = 0;


    bool reversing = false;
    double angle;
    int activeState = 0;
    double stable_speed;
    public:
        basic_area_awareness()
        {
           ROS_INFO_STREAM("Hello World");
            ROS_INFO("Hello World with just info");
     
            n = ros:: NodeHandle("~");
        
            std::string scan_topic, drive_topic, odom_topic;//,mux_topic;

        n.getParam("baa_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("scan_topic", scan_topic);

        // get car parameters
        
        n.getParam("stable_speed", stable_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("avoiding_obstacle",avoiding_obstacle);
        //   //Front vision cone
        // n.getParam("min_los_p",min_los_p);
        // n.getParam("max_los_p",max_los_p);
        // //Back vision cone
        // n.getParam("min_los_n",min_los_n);
        // n.getParam("max_los_n",max_los_n);
    // int narrowFrontLeft_a[15];
    // int narrowBackLeft_a[16];
    // int wideLeft_a[76];
    // int narrowFrontRight_a[15];
    // int narrowBackRight_a[16];
    // int wideRight_a[76];
    for(int i = 0; i < 15; i++)
    {
        narrowFrontLeft[i] = 0;
        narrowFrontRight[i] = 0;
    }    
    for(int i = 0; i < 16; i++)
    {
        narrowBackLeft[i] = 0;
        narrowBackRight[i] = 0;
    }
    for(int i = 0; i< 76; i++)
    {
        wideLeft[i] = 0;
        wideRight[i] = 0;
    }
    // narrowFrontLeft = narrowFrontLeft_a;
    // narrowFrontRight = narrowFrontRight_a;
    // narrowBackLeft = narrowBackLeft_a;
    // narrowBackRight = narrowBackRight_a;
    // wideLeft = wideLeft_a;
    // wideright = wideRight_a



        //n.getParam("mux_topic", mux_topic);//might delete

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
            //Make a publisher for mux messages
        // mux_pub = n.advertise<std_msgs::Int32MultiArray>(mux_topic, 10);
        
       // Start subscribers to listen to laser scan
         laser_sub = n.subscribe(scan_topic, 1, &basic_area_awareness::laser_callback, this);
        //listen to odom messages. 
         odom_sub = n.subscribe(odom_topic, 1, &basic_area_awareness::odom_callback, this);
     
         state = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0,  .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
    //   //  odom_sub = n.subscribe(odom_topic, 1, &basic_area_awareness::odom_callback, this);

    //    // Get params for precomputation and collision detection
        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        n.getParam("tta_threshold", tta_threshold);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("width", width);
        n.getParam("wheelbase", wheelbase);
        n.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;

        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, wheelbase, width, 
                scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);
        }

  int counterToTurnBrakingOff;
  int brakecounterThreshold = 50;
    void laser_callback(const sensor_msgs::LaserScan & msg) 
        {
            if(braking)
            {
                counterToTurnBrakingOff++;
                drive_instructions(0, 0);
                if(counterToTurnBrakingOff > brakecounterThreshold )
                {
                    counterToTurnBrakingOff = 0; 
                    braking = false;
                }
            }
            else
            {
            scanForCollidables(msg);
            setCarMovementState();
            drive_instructions(forwardDirection, turnDirection);
            }
            
        }
        void odom_callback(const nav_msgs::Odometry & msg) 
        {
        // Keep track of state to be used elsewhere
        state.velocity = msg.twist.twist.linear.x;
        state.angular_velocity = msg.twist.twist.angular.z;
        state.x = msg.pose.pose.position.x;
        state.y = msg.pose.pose.position.y;
        }
      
double flexibleThreshold = 2;
double constantThreshold = 2;
double straightThreshold = 1.25;
double turnThreshold = 1;
    void setCarMovementState()
    {
        int averageNarrowForwardValue = minNFvalL;
        int averageNarrowBehindValue = minNBvalL;
        int averageWideValue = minWvalL;
        if(minNFvalL >= minNFvalR)
        averageNarrowForwardValue= (minNFvalR + minNFvalL)/2;
        if(minNBvalL >= minNBvalR)
        averageNarrowBehindValue = (minNBvalR + minNBvalL)/2;
        if(minWvalL >= minWvalR)
        averageWideValue = (minWvalR + minWvalR)/2;

        // std::cout<<"Forward \t\t Behind \t\t Wide \t\t" << endl;
        // std::cout<<averageNarrowForwardValue<< " \t\t "<< averageNarrowBehindValue << " \t\t "<< averageWideValue << " \t\t"<<endl;

        //std::cout << "Forward Value is: " << averageNarrowForwardValue<< " and the reverse value is: " << averageNarrowBehindValue << endl;
        
        //I chose a switch because the instructions should execute at the same speed as one another
        //If-else statements will slow down the more they have to go through  
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        switch(activeState)
        {
            case 1: //moving forward
                //ROS_INFO_STREAM("Moving forward");
                if(averageNarrowForwardValue < straightThreshold)
                {
                    //brake for a sec

                    braking = true;
                    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    if(minWvalL > turnThreshold)
                    {
                        activeState = 3;
                        forwardDirection = 1; turnDirection = -1;
                    }
                    else if (minWvalR > turnThreshold)
                    {
                        activeState = 4;
                        forwardDirection = 1; turnDirection = 1;
                    }
                    else if(averageNarrowBehindValue > straightThreshold)
                    {
                        activeState = 2;
                        forwardDirection = -1; turnDirection = 0;
                    }
                    else if (minNBvalR > turnThreshold)
                    {
                        activeState = 5;
                        forwardDirection = -1; turnDirection = -1;
                    }
                    else if(minNBvalR > turnThreshold)
                    {
                        activeState = 6;
                        forwardDirection = -1; turnDirection = 1;
                    }
                    else
                    {
                        activeState = 0;
                        braking = true;
                    }
                }
            break;
            case 2://reversing
                if(averageNarrowBehindValue < straightThreshold)
                {
                    //brake for a sec
                    braking = true;
                    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                     if (minNBvalL > turnThreshold)
                    {
                        activeState = 5;
                        forwardDirection = -1; turnDirection = -1;
                    }
                    else if(minNBvalR > turnThreshold)
                    {
                        activeState = 6;
                        forwardDirection = -1; turnDirection = 1;
                    }
                    else if(averageNarrowForwardValue > straightThreshold)
                    {
                        activeState = 1;
                        forwardDirection = 1; turnDirection = 0;
                    }
                    else if(minWvalL > turnThreshold)
                    {
                        activeState = 3;
                        forwardDirection = 1; turnDirection = -1;
                    }
                    else if (minWvalR > turnThreshold)
                    {
                        activeState = 4;
                        forwardDirection = 1; turnDirection = 1;
                    }
                    else
                    {
                        activeState = 0;
                        braking = true;
                    }
                }

            break;
            case 3://turning left
                if(minWvalL < straightThreshold)
                {
                    //brake for a sec
                    braking = true;
                    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    if(averageNarrowForwardValue > straightThreshold)
                    {
                        activeState = 1;
                        forwardDirection = 1; turnDirection = 0;
                    }
                    else if (minWvalR > turnThreshold)
                    {
                        activeState = 4;
                        forwardDirection = 1; turnDirection = 1;
                    }
                    else if(averageNarrowBehindValue > straightThreshold)
                    {
                        activeState = 2;
                        forwardDirection = -1; turnDirection = 0;
                    }
                    else if (minNBvalL > turnThreshold)
                    {
                        activeState = 5;
                        forwardDirection = -1; turnDirection = -1;
                    }
                    else if(minNBvalR > turnThreshold)
                    {
                        activeState = 6;
                        forwardDirection = -1; turnDirection = 1;
                    }
                    else
                    {
                        activeState = 0;
                        braking = true;
                    }
                }
            break;
            case 4://turning right
                if(minWvalR < straightThreshold)
                {
                    //brake for a sec
                    braking = true;
                    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    if(averageNarrowForwardValue > straightThreshold)
                    {
                        activeState = 1;
                        forwardDirection = 1; turnDirection = 0;
                    }
                    else if (minWvalL > turnThreshold)
                    {
                        activeState = 3;
                        forwardDirection = 1; turnDirection = -1;
                    }
                    else if(averageNarrowBehindValue > straightThreshold)
                    {
                        activeState = 2;
                        forwardDirection = -1; turnDirection = 0;
                    }
                    else if (minNBvalL > turnThreshold)
                    {
                        activeState = 5;
                        forwardDirection = -1; turnDirection = -1;
                    }
                    else if(minNBvalR > turnThreshold)
                    {
                        activeState = 6;
                        forwardDirection = -1; turnDirection = 1;
                    }
                    else
                    {
                        activeState = 0;
                        braking = true;
                    }
                }
            break;
            case 5://reversing left
                if(minNBvalL < turnThreshold)
                {
                    //brake for a sec
                    braking = true;
                    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    if(averageNarrowForwardValue > straightThreshold)
                    {
                        activeState = 1;
                        forwardDirection = 1; turnDirection = 0;
                    }
                    else if (minNBvalR > turnThreshold)
                    {
                        activeState = 6;
                        forwardDirection = -1; turnDirection = 1;
                    }
                    else if(averageNarrowBehindValue > straightThreshold)
                    {
                        activeState = 2;
                        forwardDirection = -1; turnDirection = 0;
                    }
                    else if (minWvalL > turnThreshold)
                    {
                        activeState = 3;
                        forwardDirection = 1; turnDirection = -1;
                    }
                    else if(minWvalR > turnThreshold)
                    {
                        activeState = 4;
                        forwardDirection = 1; turnDirection = 1;
                    }
                    else
                    {
                        activeState = 0;
                        braking = true;
                    }
                }
            break;
            case 6://reversing right
                if(minNBvalR < turnThreshold)
                {
                    //brake for a sec
                    braking = true;
                    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    if(averageNarrowForwardValue > straightThreshold)
                    {
                        activeState = 1;
                        forwardDirection = 1; turnDirection = 0;
                    }
                    else if (minNBvalL > turnThreshold)
                    {
                        activeState = 5;
                        forwardDirection = -1; turnDirection = -1;
                    }
                    else if(averageNarrowBehindValue > straightThreshold)
                    {
                        activeState = 2;
                        forwardDirection = -1; turnDirection = 0;
                    }
                    else if (minWvalL > turnThreshold)
                    {
                        activeState = 3;
                        forwardDirection = 1; turnDirection = -1;
                    }
                    else if(minWvalR > turnThreshold)
                    {
                        activeState = 4;
                        forwardDirection = 1; turnDirection = 1;
                    }
                    else
                    {
                        activeState = 0;
                        braking = true;
                    }
                }
            break;
            default:
                //Check front, if front is clear just go forward. 
                    //if front isn't clear check back
                    //if back is clear reverse
                        //if back isn't clear then check your left
                            //if left is clear turn left 
                            //if left isn't clear check right
                            //if right is clear, turn right 
                            //if neither are clear check back right
                            //if back right is clear turn right 
                            //if back right isn't clear check back left
                            //if back left is clear reverse left 
                            //if back left isn't clear reduce threshold and check again.   
                if(averageNarrowForwardValue > flexibleThreshold)
                {
                    flexibleThreshold = constantThreshold;
                    activeState = 1;
                        forwardDirection = 1; turnDirection = 0;
                }
                else if(averageNarrowBehindValue > flexibleThreshold)
                {
                    flexibleThreshold = constantThreshold;
                    activeState = 2;
                        forwardDirection = -1; turnDirection = 0;
                }
                else if (averageWideValue > flexibleThreshold)
                {
                    if(minWvalL > flexibleThreshold)
                    {
                        flexibleThreshold = constantThreshold;
                        activeState = 3;
                        forwardDirection = 1; turnDirection = -1;
                    }
                    else if (minWvalR > flexibleThreshold)
                    {
                        flexibleThreshold = constantThreshold;
                        activeState = 4;
                        forwardDirection = 1; turnDirection = 1;
                    }
                }
                else if (minNBvalL >= flexibleThreshold)
                {
                    flexibleThreshold = constantThreshold;
                    activeState = 5;
                        forwardDirection = -1; turnDirection = -1;
                }
                else if (minNBvalR >= flexibleThreshold)
                {
                    flexibleThreshold = constantThreshold;
                    activeState = 6;
                        forwardDirection = -1; turnDirection = 1;
                }
                else 
                {
                    if(flexibleThreshold >=0.3)
                    {flexibleThreshold -=0.2;}
                    else
                    {
                        //ROS_INFO_STREAM("OH NO I'M TRAPPED AND I HAVEN'T EVEN MOVED YET FROM STATE 0");
                    }
                }
            break;

        }

    }

   int forwardDirection = 0;
   int turnDirection = 0;
    void drive_instructions(int forwardDir, int turnDir)
    {
    // initialize message to be published

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = forwardDir*stable_speed;


        /// STEERING ANGLE CALCULATION
        double turn = turnDir*max_steering_angle / 2.0;

        drive_msg.steering_angle = turn ;

        // reset previous desired angle
        prev_angle = drive_msg.steering_angle;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }
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
        // narrowFrontLeft = new int[15];
        // narrowBackLeft = new int[16];
        // wideLeft = new int[76];
        // narrowFrontRight = new int[15];
        // narrowBackRight = new int[16];
        // wideRight = new int[76];
         minNFvalL = 100;
         minNFvalR = 100;
         minNBvalL = 100;
         minNBvalR = 100;
         minWvalL = 100;
         minWvalR = 100;
        int messages[1080];
        
        int nfIndexLeft[15] = 
        {526,527,528,529,530,531,532,533,534,535,536,537,538,539,540};
        
        //nfIndex = nfIndex1;
        int nbIndexLeft[16] =
        {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
        
        //nbIndex = nbIndex1;
        int wIndexLeft[76] =
        {450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,
            465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,
            485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,
            505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,
            525
        };
        int nfIndexRight[15] = 
        {541,542,543,544,545,546,547,548,549,550,551,552,553,554,555};
        //nfIndex = nfIndex1;
        int nbIndexRight[16] =
        {1065,1066,1067,1068,1069,1070,1071,1072,1073,1074,1075,1076,1077,1078,1079,1080};
        
        //nbIndex = nbIndex1;
        int wIndexRight[76] =
        {
           556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,
           572,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,
           588,589,590,591,592,593,594,595,596,597,598,599,600,601,602,603,
           604,605,606,607,608,609,610,611,612,613,614,615,616,
           617,618,619,620,621,622,623,624,625,626,627,628,629,630,631
        };

    //   calculateTimeAndGiveMinimumValue(tta_threshold, 15, nfIndexLeft,
    //     narrowFrontLeft,nfCountL,minNFvalL, msg);
        // calculateTimeAndGiveMinimumValue(tta_threshold, 15, nfIndexRight,
        // narrowFrontRight,nfCountR,minNFvalR, msg);
        //  calculateTimeAndGiveMinimumValue(tta_threshold, 16, nbIndexLeft,
        // narrowBackLeft,nbCountL,minNBvalL, msg);
        // calculateTimeAndGiveMinimumValue(tta_threshold, 16, nbIndexRight,
        // narrowBackRight,nbCountR,minNBvalR, msg);
        //  calculateTimeAndGiveMinimumValue(tta_threshold, 76, wIndexLeft,
        // wideLeft,wCountL,minWvalL, msg);
        // calculateTimeAndGiveMinimumValue(tta_threshold, 76, wIndexRight,
        // wideRight,wCountR,minWvalR, msg);
    //ROS_INFO_STREAM(minNFvalL);
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
            if(time_to_intercept <= tta_threshold && time_to_intercept > 0)
            {
                narrowFrontLeft[nfCountL] = time_to_intercept;
                nfCountL++;
                if(minNFvalL >= time_to_intercept) 
                {
                    minNFvalL = time_to_intercept; 
                }
            }
        } 
        nfCountR = 0;
        for(int i : nfIndexRight)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold && time_to_intercept > 0)
            {
                
                narrowFrontRight[nfCountR] = time_to_intercept;
                if(minNFvalR >= time_to_intercept) 
                {
                    minNFvalR = time_to_intercept; 
                    
                }
                nfCountR++;
              //  std::cout<<"near front right is currently seeing: " << time_to_intercept << endl;
            }
        } 
      
        nbCountL = 0;
        for(int i : nbIndexLeft)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold && time_to_intercept > 0)
            {
                narrowBackLeft[nbCountL] = time_to_intercept;
                if(minNBvalL >= time_to_intercept) 
                {
                    minNBvalL = time_to_intercept;
                } 
               nbCountL++;
            }
        }
        nbCountR = 0;
        for(int i : nbIndexRight)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold && time_to_intercept > 0)
            {
                narrowBackRight[nbCountR] = time_to_intercept;
                if(minNBvalR >= time_to_intercept) 
                {
                    minNBvalR = time_to_intercept; 
                }
               nbCountR++;
            }
        }
       
        wCountL = 0;
        for(int i : wIndexLeft)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold && time_to_intercept > 0)
            {
                wideLeft[wCountL] = time_to_intercept;
                if(minWvalL >= time_to_intercept) 
                {
                    minWvalL = time_to_intercept; 
                }
                wCountL++;
            }
        }
        wCountR = 0;
        for(int i : wIndexRight)
        {
            angle = msg.angle_min + i*msg.angle_increment;
            double proj_velocity = state.velocity * cosines[i];
            double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
            if(time_to_intercept <= tta_threshold && time_to_intercept > 0)
            {
                wideRight[wCountR] = time_to_intercept;
                if(minWvalR >= time_to_intercept) 
                {
                    minWvalR = time_to_intercept; 
                }
                wCountR++;
            }
        }
       
       // ROS_INFO_STREAM("Finished checking collision stuff");
    }



//   int findMinValue(int minVal, int* selectionToCompare, int selectionCount)
//         {
//             minVal = 100;
//             if(selectionCount > 0)
//             {
//                 for(int i = 0; i < selectionCount; i++)
//                 {
//                     if(selectionToCompare[i] < minVal && selectionToCompare[i] > 0)
//                     minVal = selectionToCompare[i];
//                 }
//             }
//             return minVal;
//         }
    // void los_checker()
    // {
    
    //     if(reversing)
    //         {
    //             min_los = min_los_n;
    //             max_los = max_los_n;
    //         }
    //     else
    //         {
    //             min_los = min_los_p;
    //             max_los = max_los_p;
    //         }
    // }
    // void avoidanceLogic()
    // {
    //     // findMinValue
    // }
    //     int moveState = 0;
    // void avoidance_checker(const sensor_msgs::LaserScan & msg) 
    //     {
    //         // This function calculates Time To Arrival 
    //         //to check if object should be avoided
    //         if (state.velocity != 0) 
    //         {
    //             for (size_t i = 0; i < msg.ranges.size(); i++) 
    //             {
    //                 double angle = msg.angle_min + i * msg.angle_increment;
    //                  if (angle > min_los && angle < max_los)
    //                 {

    //                 counter++;
    //                 if(counter > counter_max)
    //                 counter -= counter_max;
    //                 {
    //                 // calculate projected velocity
    //                 double proj_velocity = state.velocity * cosines[i];
    //                 double tta = (msg.ranges[i] - car_distances[i]) / proj_velocity;
                    
    //                 // Check to make sure 
    //                 if ((tta < tta_threshold) && (tta >= 0.0)) 
    //                 { 
                        
    //                     if(tta < ttr_threshold)
    //                     {
    //                      reversing = !reversing; 
    //                      direction *= -1;   
    //                     }
                        
    //                     ROS_INFO_STREAM("Avoiding obstacle");
    //                 // Go to 
    //                 avoiding_obstacle = true;
    //                 basic_area_awareness::randomMovement();
    //                 return;
    //                 }
    //                 else
    //                 {
                        
    //                     ROS_INFO_STREAM("moving straight");
                        
    //                     basic_area_awareness::publish_to_drive(baa_speed,prev_angle);
    //                     avoiding_obstacle = false;
    //                 return;
    //                 }
    //             }
    //             }
    //             }
    //         }
    //     }
 
   
    //couldn't get it to work. 
    //I have no idea how to pass references to variables and modify them in a function call.  
    // void calculateTimeAndGiveMinimumValue(double thresholdToCare, int arrSize, int listOfScanIndices, 
    // int& ListOfScansWithinThreshold, int& count, int& minimumValue, const sensor_msgs::LaserScan & msg)
    // {
    //     //int arraySize = sizeof(ListOfScansWithinThreshold)/sizeof(ListOfScansWithinThreshold[0]);
        
    //     //generate temporary arrays for the loop
    //     minimumValue = 100;
    //     double angle;
    //     count = 0;
    //    // for(std::size_t i = 0; i < listOfScanIndices.size();i++)
    //     for (int i = 0; i < arrSize; i++)
    //     {
    //         angle = msg.angle_min + listOfScanIndices[i]*msg.angle_increment;
    //         double proj_velocity = state.velocity * cosines[listOfScanIndices[i]];
    //         double time_to_intercept = (msg.ranges[listOfScanIndices[i]] -
    //          car_distances[listOfScanIndices[i]]) / proj_velocity;
    //         if(time_to_intercept <= thresholdToCare)
    //         {
    //             ListOfScansWithinThreshold[count] = time_to_intercept;
                // if(minimumValue <= time_to_intercept && time_to_intercept > 0) 
                // {
                //     minimumValue = time_to_intercept; 
                // }
                
    //             count++;
    //         }
    //     }

        
    //     // for(int i : listOfScanIndices)
    //     // {
    //     //     angle = msg.angle_min + i*msg.angle_increment;
    //     //     double proj_velocity = state.velocity * cosines[i];
    //     //     double time_to_intercept = (msg.ranges[i] - car_distances[i]) / proj_velocity;
    //     //     if(time_to_intercept <= thresholdToCare)
    //     //     {
    //     //         ListOfScansWithinThreshold[*count] = time_to_intercept;
    //     //         if(*minimumValue <= time_to_intercept) *minimumValue = time_to_intercept; 
    //     //         *count++;
    //     //     }
    //     // }
    // }
//  int direction = 1;
//     void randomMovement()
//     {
//         /// STEERING ANGLE CALCULATION
//         // random number between 0 and 1
//         double random = ((double) rand() / RAND_MAX);
//         // good range to cause lots of turning
//         double range = max_steering_angle / 2.0;
//         // compute random amount to change desired angle by (between -range and range)
//         double rand_ang = range * random - range / 2.0;

//         // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
//         random = ((double) rand() / RAND_MAX);
//         if ((random > .8) && (prev_angle != 0)) 
//         {
//             double sign_rand = rand_ang / std::abs(rand_ang);
//             double sign_prev = prev_angle / std::abs(prev_angle);
//             rand_ang *= sign_rand * sign_prev;
//         }
//         double angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle);
//         basic_area_awareness::publish_to_drive(2*direction, angle*direction);
//     }
//      void publish_to_drive(double desired_velocity, double desired_steer) 
//      {
//         // This will take in a desired velocity and steering angle and make and publish an 
//         // AckermannDriveStamped message to the /drive topic

//         // Make and publish message
//         ackermann_msgs::AckermannDriveStamped drive_st_msg;
//         ackermann_msgs::AckermannDrive drive_msg;

//         //std_msgs::Header header;
//         drive_msg.speed = desired_velocity*direction;
//         drive_msg.steering_angle = desired_steer*direction;
//         // reset previous desired angle
//         prev_angle = drive_msg.steering_angle;

//         //header.stamp = ros::Time::now();

//         //drive_st_msg.header = header;

//         // set drive message in drive stamped message
//         drive_st_msg.drive = drive_msg;

//         // publish AckermannDriveStamped message to drive topic
//         drive_pub.publish(drive_st_msg);
//     }


};
int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "basic_area_awareness");
    basic_area_awareness ba;
    ros::spin();
    return 0;
}
