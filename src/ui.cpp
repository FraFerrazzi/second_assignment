#include "ros/ros.h"
#include "second_assignment/VelService.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

// function to print on screen the options which the user can choose from
void print_ui()
{
    // User Interface
    std::cout << "Type commands on keyboard to change the robot velocity:\n\n";
    std::cout << "1: decrease linear velocity by two, angular velocity by point seven\n";
    std::cout << "2: decrease linear velocity by one, angular velocity by point three\n";
    std::cout << "3: decrease linear velocity by one half, angular velocity by little\n";
    std::cout << "4: increase linear velocity by one half, angular velocity by little\n";
    std::cout << "5: increase linear velocity by one, angular velocity by point three\n";
    std::cout << "6: increase linear velocity by two, angular velocity by point seven\n\n";
    std::cout << "0: Reset robot position\n\n";
    std::cout << "9: EXIT THE PROGRAM\n\n";
}

/* 
   Function that checks the user's input from keyboard and returns it 
 
   @return int, user's input
*/
int check_user_input()
{
    int input; // initialize variable for user input
    std::cout << "\nCommand: ";
    std::cin >> input; // getting input from user
    while(std::cin.fail()) // check that cin is an int
    {
        ROS_INFO("Error! The input given by user is not an integer number"); // Feedback to user
        std::cin.clear();
        std::cin.ignore(256,'\n');
        std::cin >> input;
    }
    return input;
}


int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Calling function for printing ui
    print_ui();

    // Create a client to control velocity of the robot
    ros::ServiceClient client_vel = nh.serviceClient<second_assignment::VelService>("/set_velocity");
    // Defining a variable srv_vel of type second_assignment::VelService
    second_assignment::VelService srv_vel;

    // Create a client to reset robot position
    ros::ServiceClient client_res = nh.serviceClient<std_srvs::Empty>("/reset_positions");
    // Defining a variable srv_res of type std_srvs::Empty
    std_srvs::Empty srv_res;

    int inputU; // initialize variable for user input

    // Infinite loop until user doesn't press 9 and ros::ok() returns true
    while(inputU != 9 && ros::ok())
    {
        int inputU = check_user_input();

        if (inputU == 0)
        {
            client_res.waitForExistence(); // make sure the service is active
            client_res.call(srv_res); // Call reset service
            std::cout << "Reset robot position\n";  // Feedback to user
        }
        else
        {
            srv_vel.request.decision = inputU;
            client_vel.waitForExistence(); // make sure the service is active
            client_vel.call(srv_vel); // Call VelService
            if (inputU == 9)
            {
                ROS_INFO("EXIT");
                exit(0);
            }
        }  
        // Clear terminal
        std::system("clear"); 
        // Calling function for UI 
        print_ui();
        // feedback to user
        ROS_INFO("Robot linear velocity: %.2f", srv_vel.response.x); 
        ROS_INFO("Robot angular velocity: %.2f", srv_vel.response.z);
        ros::spinOnce();
    }
}