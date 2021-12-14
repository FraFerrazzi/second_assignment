// to run the program is sufficient to type on terminal: roslaunch second_assignment second_ass.launch

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "second_assignment/VelService.h"

// defining the publisher
ros::Publisher pub;

// setting regions for laser scan sensor
#define RIGHT 1 
#define RIGHT_FRONT 2 
#define FRONT 3 
#define LEFT_FRONT 4 
#define LEFT 5 

// Range size is 720 (for 180 degrees)
// There are 4 values for every degree
#define RIGHT_DIM 160 // 40 degrees on the right
#define RIGHT_FRONT_DIM 140
#define FRONT_DIM 120 // 30 degrees in front of the robot
#define LEFT_FRONT_DIM 140
#define LEFT_DIM 160 // 40 degrees on the left

// Setting thresholds
#define L_DIFF_RIGHT_LEFT 0.13
#define L_M_DIFF_RIGHT_LEFT 0.2
#define M_DIFF_RIGHT_LEFT 0.32
#define H_DIFF_RIGHT_LEFT 0.39
#define THRESHOLD 1.2
#define THRESHOLD_CRASH 0.8
#define THRESHOLD_SIDE 0.5

// Setting max and min velocities
#define MAX_LIN_VEL 6
#define MIN_LIN_VEL 0.5
#define MAX_ANG_VEL 2
#define MIN_ANG_VEL 0.25

// Global variables to store the velocity
float linear_velocity = 3;
float angular_velocity = 1;

float change_lin_vel;
float change_ang_vel;

/* 
   Function that gets the minimum value among the elements of a region of a given array
 
   @param vector_ranges, the whole vector in which the distance values are stored 
   @param direction, desired section of the array to work on
   @return float, minimum value all the elements of the desired reagion of the vector
*/
float get_min_val(float vector_ranges[], int direction)
{
    float min_dist = 50;
    float right[RIGHT_DIM];
    float right_front[RIGHT_FRONT_DIM];
    float front[FRONT_DIM];
    float left_front[LEFT_FRONT_DIM];
    float left[LEFT_DIM];

    if (direction == RIGHT) // checks values of the array on the right side of the robot
    {
        float range_val[RIGHT_DIM];

        for (int i = 0; i <= RIGHT_DIM; i++)
        {
            range_val[i] = vector_ranges[i];
            if (range_val[i] < min_dist) // get min among the section
            {
                min_dist = range_val[i];
            }
        }
    }
    else if (direction == RIGHT_FRONT) // check values of the array on the right front side of the robot
    {
        float range_val[RIGHT_FRONT_DIM];

        for (int i = 160; i <= RIGHT_FRONT_DIM + 160; i++)
        {
            range_val[i - 160] = vector_ranges[i];
            if (range_val[i - 160] < min_dist) // get min among the section
            {
                min_dist = range_val[i - 160];
            }
        }
    }
    else if (direction == FRONT)  // check values of the array in front of the robot
    {
        float range_val[FRONT];

        for (int i = 300; i <= FRONT_DIM + 300; i++)
        {
            range_val[i - 300] = vector_ranges[i];
            if (range_val[i - 300] < min_dist) // get min among the section
            {
                min_dist = range_val[i - 300];
            }
        }
    }
    else if (direction == LEFT_FRONT)  // check values of the array on the left front side of the robot
    {
        float range_val[LEFT_FRONT];

        for (int i = 419; i <= LEFT_FRONT_DIM + 419; i++)
        {
            range_val[i - 419] = vector_ranges[i];
            if (range_val[i - 419] < min_dist) // get min among the section
            {
                min_dist = range_val[i - 419];
            }
        }
    }
    else if (direction == LEFT)  // check values of the array on the left side of the robot
    {
        float range_val[LEFT];

        for (int i = 559; i <= LEFT_DIM + 559; i++)
        {
            range_val[i - 559] = vector_ranges[i];
            if (range_val[i - 559] < min_dist) // get min among the section
            {
                min_dist = range_val[i - 559];
            }
        }
    }
    return min_dist; 
}

/*
   Function used to determine the direction in which the robot should move defining it's
   angular and linear velocity 
   angular.z > 0 : turn left
   angular.z < 0 : turn right
   The function executes each time a message is published on the /base_scan topic

   @param msg, is a message of type sensor_msgs/LaserScan that allowes to obtain the 
   whole ranges vector
*/
void decide_direction(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	float vector_ranges[msg->ranges.size()];
    for (int i = 0; i < msg->ranges.size(); i++) // get whole vector ranges
    {
        vector_ranges[i] = msg->ranges[i]; 
    }
    // getting minimum values in the ranges' section desired
    float min_right = get_min_val(vector_ranges, RIGHT);
    float min_right_front = get_min_val(vector_ranges, RIGHT_FRONT);
    float min_front = get_min_val(vector_ranges, FRONT);
    float min_left_front = get_min_val(vector_ranges, LEFT_FRONT);
    float min_left = get_min_val(vector_ranges, LEFT);
	
	geometry_msgs::Twist my_vel;

	// nothing is close to the robot and the robot is well centered in the circuit
	if (min_front > THRESHOLD && abs(min_left - min_right) < L_DIFF_RIGHT_LEFT && min_left_front > THRESHOLD_SIDE && min_right_front > THRESHOLD_SIDE )
    {
	    my_vel.linear.x = linear_velocity;
	    my_vel.angular.z = 0;
	}
	else
    {
        // robot is too close to an obstacle: need to avoid it
        if (min_front < THRESHOLD_CRASH)
        {
            // obstacle on the left is further than obstacle on the right
            if (min_left > min_right || min_left_front > min_right_front)
            {
                my_vel.linear.x = 0;
                my_vel.angular.z = angular_velocity; // turn left
            }
            else
            {
                my_vel.linear.x = 0;
                my_vel.angular.z = -angular_velocity; // turn right
            }
        }
        else 
        {
            // min distance from left and right is above a high threshold: should turn left and be careful
            if (min_left_front - min_right_front >= H_DIFF_RIGHT_LEFT)
            {
                my_vel.linear.x = 0.2*linear_velocity;
                my_vel.angular.z = angular_velocity; 
            }
            else if (min_right_front - min_left_front > H_DIFF_RIGHT_LEFT)
            {
                my_vel.linear.x = 0.2*linear_velocity;
                my_vel.angular.z = -angular_velocity; 
            }
            // min distance from left and right is above a midium threshold: should turn left a bit and slow down
            else if (min_left_front - min_right_front >= M_DIFF_RIGHT_LEFT)
            {
                my_vel.linear.x = 0.5*linear_velocity;
                my_vel.angular.z = 0.4*angular_velocity;
            }
            else if (min_right_front - min_left_front > M_DIFF_RIGHT_LEFT)
            {
                my_vel.linear.x = 0.5*linear_velocity;
                my_vel.angular.z = -0.4*angular_velocity;
            }
            // min distance from left and right is above a low-medium threshold: should stay as centered as possible
            else if (min_left_front - min_right_front >= L_M_DIFF_RIGHT_LEFT)
            {
                my_vel.linear.x = 0.8*linear_velocity;
                my_vel.angular.z = 0.2*angular_velocity;
            }
            else if (min_right_front - min_left_front > L_M_DIFF_RIGHT_LEFT)
            {
                my_vel.linear.x = 0.8*linear_velocity;
                my_vel.angular.z = -0.2*angular_velocity;
            }
            // min distance from left and right is above a low threshold: should slightly turn left and keep going
            else if (min_left_front - min_right_front >= L_DIFF_RIGHT_LEFT)
            {
                my_vel.linear.x = linear_velocity;
                my_vel.angular.z = 0.05*angular_velocity;
            }
            else if (min_right_front - min_left_front > L_DIFF_RIGHT_LEFT)
            {
                my_vel.linear.x = linear_velocity;
                my_vel.angular.z = -0.05*angular_velocity;
            }
            // stall situations: robot doesn't really know what to do
            else
            {
                // obstacle on left is further than obstacle on right: tun left
                if (min_left_front >= min_right_front || min_left >= min_right)
                {
                    my_vel.linear.x = 0.2;
                    my_vel.angular.z = angular_velocity;
                }
                else
                {
                    my_vel.linear.x = 0.2;
                    my_vel.angular.z = -angular_velocity;
                }
            }
        }
    }
    pub.publish(my_vel);
}

/*
   Function used to change the velocity of the robot real-time while the program 
   is running

   @param req, is the request message sent by user that will be handled by the switch function
   @param res, is the response message given by the server
   @return boolean, true if service is called
*/
bool set_velocity (second_assignment::VelService::Request &req, second_assignment::VelService::Response &res)
{
    switch(req.decision)
    {
        // decrease lin vel by two, ang vel by point seven
        case 1:
            change_lin_vel = -2;
            change_ang_vel = -0.7;
            linear_velocity += change_lin_vel;
            angular_velocity += change_ang_vel;
            break;

        // decrease lin vel by one, ang vel by point three
        case 2:
            change_lin_vel = -1;
            change_ang_vel = -0.3;
            linear_velocity += change_lin_vel;
            angular_velocity += change_ang_vel;
            break;

        // decrease lin vel by point five, ang vel by point seventeen
        case 3:
            change_lin_vel = -0.5;
            change_ang_vel = -0.17;
            linear_velocity += change_lin_vel;
            angular_velocity += change_ang_vel;
            break;

        // increase lin vel by point five, ang vel by point seventeen
        case 4:
            change_lin_vel = 0.5;
            change_ang_vel = 0.17;
            linear_velocity += change_lin_vel;
            angular_velocity += change_ang_vel;
            break;

        // increase lin vel by one, ang vel by point three
        case 5:
            change_lin_vel = 1;
            change_ang_vel = 0.3;
            linear_velocity += change_lin_vel;
            angular_velocity += change_ang_vel;
            break;

        // increase lin vel by two, ang vel by point seven
        case 6:
            change_lin_vel = 2;
            change_ang_vel = 0.7;
            linear_velocity += change_lin_vel;
            angular_velocity += change_ang_vel;
            break;

        // quit
        case 9:
            ROS_INFO("EXIT");
            sleep(1);
            exit(0);
            break;

        default:
            std::cout << "Not a valid input. Choose a number between 1 and 6!\n";
            break;
    }
    // check if linear and angular velocity are too high or too low
    if(linear_velocity > MAX_LIN_VEL)
    {
        linear_velocity = MAX_LIN_VEL;
    }
    else if (linear_velocity < MIN_LIN_VEL)
    {
        linear_velocity = MIN_LIN_VEL;
    }
    if(angular_velocity > MAX_ANG_VEL)
    {
        angular_velocity = MAX_ANG_VEL;
    }
    else if (angular_velocity < MIN_ANG_VEL)
    {
        angular_velocity = MIN_ANG_VEL;
    }
    // change lin and ang velocities with new values
    res.x = linear_velocity;
    res.z = angular_velocity;

    return true;
}


int main (int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
	ros::init(argc, argv, "controllerrobot_node");  
	ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/set_velocity", set_velocity);

    // Define a publisher that pubishes on the topic named /cmd_vel to set the robot velocity 
	pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 100); 

    // Define a subscriber that get access to the ranges vector thanks to /base_scan topic
	ros::Subscriber sub = nh.subscribe("/base_scan", 100, decide_direction);  

    // Enters a loop
    // Exit when Ctrl-C is pressed, or node is shutdown by the master
	ros::spin();
    return 0;
}