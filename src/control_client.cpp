
#include "ros/ros.h"
#include "arm_control/move_request.h"
#include "arm_control/move_confirmation.h"
#include <cstdlib>
#include <iostream>

// Once trajectory has been plotted, user will confirm whether execute it
// 1 = successful, 2 = arm movement cancelled, 3 = arm movement encountered error, 4 = server unable to connect
int confirmation_function()
{    
    ros::NodeHandle n;
    ros::ServiceClient client_confirm = n.serviceClient<arm_control::move_confirmation>("confirm_coord");
    arm_control::move_confirmation srv2; // Tell to arm to execute trajectory and then receive confirmation when it has returned to starting position
    
    char decision;
    while(!(decision == 'y' || decision == 'n'))
    {
        std::cout << "Would you like to execute trajectory? (y/n) ";
        std::cin >> decision;
        ROS_INFO("Input: %c", decision);
        //std::cout << "keyboard input: " << decision << "\n"; // Test
    }

    if(decision == 'y')
    {
        srv2.request.head_to_position = true;
        if(client_confirm.call(srv2))
        {
            if(srv2.response.finished_at_start_point)
            {
                return 1;
            }
            else if(!srv2.response.finished_at_start_point)
            {
                return 3;
            }
        }
        else
        {
            return 4;
        }
    }
    else if(decision == 'n')
    {
        return 2;
    }

    return 0;
}

int main(int argc, char **argv)
{
  // Initialise ROS node
  ros::init(argc, argv, "control_client_node");
  ROS_INFO("Please enter coordinates for arm movement and confirmation for execution.");

  // Initialize node
  ros::NodeHandle n;

  // Start (looking for) service
  ros::ServiceClient client_request = n.serviceClient<arm_control::move_request>("take_coord");
  arm_control::move_request srv1; // Provide coordinates and then receive confirmation whether it is possible

  while(true)
  {
    float x, y, z;
    
    std::cout << "Enter the X-coord: ";
    std::cin >> x;
    std::cout << "Enter the Y-coord: ";
    std::cin >> y;
    std::cout << "Enter the Z-coord: ";
    std::cin >> z;
    
    //std::cout << x << y << z << std::endl;
    //std::cout << ros::service::exists("xarm_pose_plan", true);

    srv1.request.x = x;
    srv1.request.y = y;
    srv1.request.z = z;

    if (client_request.call(srv1)) // Ensure that first service call is successful, also "executes" the call
    {
        if(srv1.response.confirmation) // If provided coordinates is confirmed to be possible
        {
            ROS_INFO("TEST: Connection established");
            int proceed;
            ROS_INFO("Movement possibility: %s", srv1.response.confirmation ? "True" : "False");
            proceed = confirmation_function();
            std::cout << "TEST: confirmation_function return value is " << proceed << "\n";
            if(proceed == 1)
            {
                std::cout << "Movement completed, please enter another set of coordinates.\n";
            }
            else if (proceed == 2)
            {
                std::cout << "Movement cancelled, please enter another set of coordinates.\n";
            }
            else if (proceed = 3)
            {
                ROS_ERROR("Arm encountered error, please inspect it");
                return 1;
            }
            else
            {
                ROS_ERROR("Failed to call service confirm_coord");
                return 1;
            }
        }
        else // Restart the while loop if coordinates are not valid
        {
            std::cout << "The provided coordinates were not valid, please try again.\n";
            // command to restart
        }
    }
    else // If unable to connect to both services, return an error message
    {
        ROS_ERROR("Failed to call service take_coord");
        return 1;
    }

  }

  return 0;
}
