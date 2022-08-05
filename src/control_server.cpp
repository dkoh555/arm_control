
#include "ros/ros.h"
#include "arm_control/move_request.h"
#include "arm_control/move_confirmation.h"

#include <xarm_planner/pose_plan.h>
#include <xarm_planner/single_straight_plan.h>
#include <xarm_planner/exec_plan.h>

#include <gripper_control/grip.h>

#include <iostream>

// Factor in input coordinate difference due to xArm base coordinates
float base_x = 0.0;
float base_y = 0.0;
float base_z = 0.372;

// Initialise coordinate variables
float x_coord;
float y_coord;
float z_coord;

// To track whether arm is set to its start position at startup
//bool calibrated = false;

bool grip_command(int dec)
{
  ros::NodeHandle n6;
  ros::ServiceClient client_request6 = n6.serviceClient<gripper_control::grip>("gripper");
  gripper_control::grip srv8;

  client_request6.waitForExistence();

  srv8.request.decision = dec;

  if(client_request6.call(srv8))
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

bool execute(void)
{
  ros::NodeHandle n5;
  ros::ServiceClient client_request5 = n5.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");
  xarm_planner::exec_plan srv7;

  srv7.request.exec = true;

  if(client_request5.call(srv7))
  {
    if(srv7.response.success)
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

bool return_to_start(void)
{
  ros::NodeHandle n4;
  ros::ServiceClient client_request4 = n4.serviceClient<xarm_planner::pose_plan>("xarm_pose_plan");
  xarm_planner::pose_plan srv6;

  client_request4.waitForExistence();

  // Raising xArm after picking up trash
  srv6.request.target.position.x  = base_x + 0.2;
  srv6.request.target.position.y  = base_y + 0.0;
  srv6.request.target.position.z  = base_z + 0.5;

  srv6.request.target.orientation.x  = 1.0;
  srv6.request.target.orientation.y  = 0.0;
  srv6.request.target.orientation.z  = 0.0;
  srv6.request.target.orientation.w  = 0.0;

  grip_command(1);

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

      srv7.request.target.orientation.x  = 0.985;
      //srv7.request.target.orientation.x  = 0.0;
      //srv7.request.target.orientation.x  = 0.0;
      srv7.request.target.orientation.y  = 0.0;
      //srv7.request.target.orientation.y  = 0.985;
      //srv7.request.target.orientation.y  = 0.924;
      srv7.request.target.orientation.z  = 0.174;
      //srv7.request.target.orientation.z  = 0.0;
      //srv7.request.target.orientation.z  = 0.0;
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

bool test1(arm_control::move_request::Request  &req,
         arm_control::move_request::Response &res)
{
  
  
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
  //std::cout << "Input: [" << x_coord << ", " << y_coord << ", " << z_coord << "]" << std::endl;

  if(client_request1.call(srv3))
  {
    if(srv3.response.success)
    {
      ROS_INFO("TEST: Trajectory is valid");
      res.confirmation = true;
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
    //ROS_ERROR("Failed to call service pose_plan to arm");
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
  // Initialise node
  ros::init(argc, argv, "control_server_node");
  ros::NodeHandle n;

  // Set arm to start position
  return_to_start();
  ROS_INFO("ARM SET TO START POSITION");


  // Initialise services to receive both client requests
  ros::ServiceServer service1 = n.advertiseService("take_coord", test1);
  ros::ServiceServer service2 = n.advertiseService("confirm_coord", test2);
  
  ROS_INFO("Standing by for move request and confirmation.");

  ros::spin();

  return 0;
}
