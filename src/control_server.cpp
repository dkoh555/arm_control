/*
Okay so this code might be a bit chonky ngl.

But basically what control_server does is...
1. Retrieve coordinates via service message (from either the object detection node (bz_detect.py or detect.py) or manually entered (control_client.cpp))
2. Transmits the coordinates to the xarm_planner node
3. If the xarm_planner comes back stating the coordinates are valid (possible for the xarm to retrieve from), control_server awaits movement confirmation
4. Once confirmation is received, the confirmation is transmitted to the xarm_planner node (basically to actually move the xarm)
5. Once xarm is in position, control_server instructs gripper to close
6. Once gripper is closed, control_server will instruct xarm to raise itself
7. xarm will be instructed to move above the trash bin (grey box in Rviz)
8. Once above the trash bin, gripper will be instructed to open (and drop object into the bin)
9. Once dropped, control_server will instruct xarm to return to orginal position
10. Once in original position, control_server will instruct xarm to adjust to original angle (for optimum camera angle)
11. Rinse and repeat

Read through this code and follow along my comments, hopefully I can explain why I do what I do.
Or just ask Hans, he's a literal god amongst men.

Okay and also, please have a BASIC knowledge of ROS otherwise even I can't help you (and I'm already very basic lmao).
*/

// Import the required libraries (which includes ROS, duh)
#include "ros/ros.h"
#include <iostream>

// Import the required topic and service messages
// For the xarm
#include "arm_control/move_request.h"
#include "arm_control/move_confirmation.h"
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/single_straight_plan.h>
#include <xarm_planner/exec_plan.h>
//For the gripper
#include <gripper_control/grip.h>

// Initialising some basic variables to be used later
// To factor in the coordinate difference due to position of the xarm relative to the ground
float base_x = 0.0;
float base_y = 0.0;
float base_z = 0.372;

// Initialise coordinate variables for later use
float x_coord;
float y_coord;
float z_coord;

// Sends out a service message to the gripper control node to open or close depending on the value of variable dec
bool grip_command(int dec)
{
  // Basic node initialization commands to prepare control_server for service message request sending
  ros::NodeHandle n6;
  ros::ServiceClient client_request6 = n6.serviceClient<gripper_control::grip>("gripper"); // "gripper" is the service that the service message will be sent to
  gripper_control::grip srv8;

  client_request6.waitForExistence(); // Tells the program to wait for the existance of the service before proceeding

  srv8.request.decision = dec; // Assigning the decision variable in the grip service message request to the value of dec

  if(client_request6.call(srv8)) // This is where the service message is actually transmitted
  {
    ROS_INFO("Gripper command completed");
  }
  else // If unable to connect to service, return an error message
  {
    ROS_ERROR("Failed to call service grip to gripper");
    return false;
  }

  return true;
}

// Tells the xarm_planner to execute the previosly input and validated movement plan
bool execute(void)
{
  ros::NodeHandle n5;
  ros::ServiceClient client_request5 = n5.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");
  xarm_planner::exec_plan srv7;
  
  client_request5.waitForExistence();

  srv7.request.exec = true;

  if(client_request5.call(srv7))
  {
    if(srv7.response.success) // Whenever a service message is sent, the sender program suspends until it receives a response from the receiver
    {
      ROS_INFO("TEST: Arm movement executed");
      return true;
    }
    else
    {
      ROS_ERROR("Arm encountered problems");
      return false;
    }
  }
  
  else // If unable to connect to service, return an error message
  {
    ROS_ERROR("Failed to call service exec_plan to arm");
    return false;
  }

  return true;
}

//  Instructs the arm to return to its starting position and angle
bool return_to_start(void)
{
  ros::NodeHandle n4;
  ros::ServiceClient client_request4 = n4.serviceClient<xarm_planner::pose_plan>("xarm_pose_plan");
  xarm_planner::pose_plan srv6;

  client_request4.waitForExistence();

  // These are the coordinates being assigned to the pose_plan message to be sent to the xarm_planner
  // Raising xArm after picking up trash
  srv6.request.target.position.x  = base_x + 0.2;
  srv6.request.target.position.y  = base_y + 0.0;
  srv6.request.target.position.z  = base_z + 0.5;

  srv6.request.target.orientation.x  = 1.0;
  srv6.request.target.orientation.y  = 0.0;
  srv6.request.target.orientation.z  = 0.0;
  srv6.request.target.orientation.w  = 0.0;

  grip_command(1); // This is the gripper function

  if(client_request4.call(srv6))
  {
    if(srv6.response.success)
    {
      ROS_INFO("TEST: return path P3 successful");
      execute();
      
      ros::NodeHandle n4;
      ros::ServiceClient client_request5 = n4.serviceClient<xarm_planner::single_straight_plan>("xarm_straight_plan");
      xarm_planner::single_straight_plan srv7;

      // Setting xArm to optimum camera angle
      srv7.request.target.position.x  = base_x + 0.2;
      srv7.request.target.position.y  = base_y + 0.0;
      srv7.request.target.position.z  = base_z + 0.5;

      //srv7.request.target.orientation.x  = 1;
      srv7.request.target.orientation.x  = 0.985;
      //srv7.request.target.orientation.x  = 0.0;
      //srv7.request.target.orientation.x  = 0.0;
      //srv7.request.target.orientation.y  = 0.0;
      srv7.request.target.orientation.y  = 0.0;
      //srv7.request.target.orientation.y  = 0.985;
      //srv7.request.target.orientation.y  = 0.924;
      //srv7.request.target.orientation.z  = 0.0;
      srv7.request.target.orientation.z  = 0.174;
      //srv7.request.target.orientation.z  = 0.0;
      //srv7.request.target.orientation.z  = 0.0;
      //srv7.request.target.orientation.w  = 0.0;
      srv7.request.target.orientation.w  = 0.0;
      //srv7.request.target.orientation.w  = 0.174;
      //srv7.request.target.orientation.w  = 0.383;
      
      if(client_request5.call(srv7))
      {
        if(srv6.response.success)
        {
          ROS_INFO("TEST: angle reset successful");
          execute();
          return true;
        }
        else
        {
          ROS_ERROR("Arm angle reset encountered problems");
          return false;
        }
      }
      else
      {
        ROS_ERROR("Failed to call service single_straight_plan to arm");
        return false;
      }
    }
    else
    {
      ROS_ERROR("Arm return path P3 encountered problems");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service pose_plan to arm");
    return false;
  }
}

//  After grabbing the object, instructs the arm to drop it into the trash bin
bool keep_trash(void)
{
  ros::NodeHandle n3;
  ros::ServiceClient client_request3 = n3.serviceClient<xarm_planner::pose_plan>("xarm_pose_plan");
  xarm_planner::pose_plan srv5;

  // Raising xArm after picking up trash
  srv5.request.target.position.x  = base_x + x_coord;
  srv5.request.target.position.y  = base_y + y_coord;
  srv5.request.target.position.z  = base_z + 0.2;

  srv5.request.target.orientation.x  = 1.0;
  srv5.request.target.orientation.y  = 0.0;
  srv5.request.target.orientation.z  = 0.0;
  srv5.request.target.orientation.w  = 0.0;

  if(client_request3.call(srv5))
  {
    if(srv5.response.success)
    {
      ROS_INFO("TEST: return path P1 successful");
      
      execute();

      // Moving xArm above trashbin to drop trash
      srv5.request.target.position.x  = base_x - 0.44;
      if(y_coord >= 0)
      {
        srv5.request.target.position.y  = base_y + 0.05;
      }
      else
      {
        srv5.request.target.position.y  = base_y - 0.05;
      }
      srv5.request.target.position.z  = base_z + (0.8 - base_z);

      srv5.request.target.orientation.x  = 0.0;
      srv5.request.target.orientation.y  = 1.0;
      srv5.request.target.orientation.z  = 0.0;
      srv5.request.target.orientation.w  = 0.0;

      if(client_request3.call(srv5))
      {
        if(srv5.response.success)
        {
          ROS_INFO("TEST: return path P2 successful");
          execute();

          grip_command(1);

          return true;
        }
        else
        {
          ROS_ERROR("Arm return path P2 encountered problems");
          return false;
        }
      }
      else
      {
        ROS_ERROR("Failed to call service pose_plan to arm");
        return false;
      }
      
    }
    else
    {
      ROS_ERROR("Arm return path P1 encountered problems");
      return false;
    }
  }
  else // If unable to connect to service, return an error message
  {
    ROS_ERROR("Failed to call service pose_plan to arm");
    return false;
  }
}

// This function uses data from a service message that control_server RECEIVES, so take note of the req and res variables
bool test1(arm_control::move_request::Request  &req,
         arm_control::move_request::Response &res)
{
  
  // Assigning the different coord variables the corresponding values from the received message
  x_coord = req.x;
  y_coord = req.y;
  z_coord = req.z;

  ros::NodeHandle n1;
  ros::ServiceClient client_request1 = n1.serviceClient<xarm_planner::pose_plan>("xarm_pose_plan");
  xarm_planner::pose_plan srv3;

  srv3.request.target.position.x  = x_coord;
  srv3.request.target.position.y  = y_coord;
  srv3.request.target.position.z  = z_coord;

  srv3.request.target.orientation.x  = 1.0;
  srv3.request.target.orientation.y  = 0.0;
  srv3.request.target.orientation.z  = 0.0;
  srv3.request.target.orientation.w  = 0.0;

  ROS_INFO("Input: [%f, %f, %f]", x_coord, y_coord, z_coord);

  if(client_request1.call(srv3))
  {
    if(srv3.response.success)
    {
      ROS_INFO("TEST: Trajectory is valid");
      res.confirmation = true; // Returning a response message containing the true boolean
    }
    else
    {
      ROS_ERROR("Plotted path is invalid, please enter a new set of coordinates");
      res.confirmation = false;
    }
  }
  
  else // If unable to connect to service, return an error message
  {
    ROS_ERROR("Plotted path is invalid, please enter a new set of coordinates");
    res.confirmation = false;
  }

  return true;
}

bool test2(arm_control::move_confirmation::Request  &req,
         arm_control::move_confirmation::Response &res)
{
  bool dec = req.head_to_position;
  
  ros::NodeHandle n2;
  ros::ServiceClient client_request2 = n2.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");
  xarm_planner::exec_plan srv4;

  srv4.request.exec = dec;

  if(client_request2.call(srv4))
  {
    if(srv4.response.success)
    {
      ROS_INFO("TEST: Arm trajectory execution success");

      grip_command(0);
      
      bool success = false;
      if(keep_trash())
      {
        success = return_to_start();
      }

      res.finished_at_start_point = success;
    }
    else
    {
      ROS_ERROR("Arm encountered problems");
      res.finished_at_start_point = false;
    }
  }
  
  else // If unable to connect to service, return an error message
  {
    ROS_ERROR("Failed to call service exec_plan to arm");
    res.finished_at_start_point = false;
  }

  return true;
}

int main(int argc, char **argv)
{
  // Initialise the control_server as a node, THIS IS IMPORTANT
  ros::init(argc, argv, "control_server_node");
  ros::NodeHandle n;

  // Set arm to start position upon running the control_server program
  return_to_start();
  ROS_INFO("ARM SET TO START POSITION");


  // Initialise services advertising, so that control_server is able receive client service message requests
  ros::ServiceServer service1 = n.advertiseService("take_coord", test1); // the function test1 will be run when it receives a service message from the service "take_coord"
  ros::ServiceServer service2 = n.advertiseService("confirm_coord", test2);
  
  ROS_INFO("Standing by for move request and confirmation.");

  ros::spin(); // this tells the control_server program to NOT stop running and shut down

  return 0;
}
