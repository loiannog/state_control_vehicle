// Regular Includes
#include <memory>
#include <math.h>
#include <iostream>

// ROS Related Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

// Quadrotor Control Includes
#include <mav_manager/manager.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>

// Local Includes
// #include "state_control.h"
#include "nano_kontrol2.h"
#include <trajectory/trajectory.h>

using namespace std;

#define RED "\e[91m"
#define GREEN "\e[92m"
#define YELLOW "\e[93m"
#define BLUE "\e[94m"
#define MAGENTA "\e[95m"
#define CYAN "\e[96m"
#define RESET "\e[0m"

std::shared_ptr<MAVManager> mav;
ros::Publisher des_odom_pub;

// State machine
enum state_enum
{
  ESTOP,
  INIT,
  TAKEOFF,
  HOVER,
  HOME,
  LINE_TRACKER,
  VELOCITY_TRACKER,
  VISION_CONTROL,
  LAND,
  PREP_TRAJ,
  TRAJ,
  NONE,
};
static enum state_enum state_ = INIT;

// Variables and parameters
double xoff, yoff, zoff, yaw_off;

// =======================
// Stuff for trajectory
// =======================
Trajectory traj;
quadrotor_msgs::PositionCommand traj_goal;
static std::string traj_filename;
static int traj_num_ = 0;
static int max_traj_num = 0;
static bool play_button_pressed = false;
// =======================

// Publishers & services
static ros::Publisher pub_traj_signal_;
static ros::Publisher pub_traj_num_;

// Quadrotor Pose
static geometry_msgs::Point goal;
nav_msgs::Odometry odom_des;

// Function Prototypes
double norm(const geometry_msgs::Point &a, const geometry_msgs::Point &b);

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->buttons[estop_button])
  {
    // Publish the E-Stop command
    if (mav->estop())
    {
      state_ = ESTOP;
      return;
    }
  }
  if(msg->buttons[eland_button])
  {
    if (mav->eland())
    {
      state_ = LAND;
      return;
    }
  }

  play_button_pressed = msg->buttons[play_button];

  if (state_ == ESTOP)
    return;

  if(state_ == INIT)
  {
    // Motors on (Rec)
    if(msg->buttons[motors_on_button])
      mav->set_motors(true);

    // Take off (Play)
    if(msg->buttons[play_button])
    {
      if (!mav->motors())
        ROS_INFO("You must turn on the motors first...");
      else
      {
        ROS_INFO("Initiating launch sequence...");
        if(mav->takeoff())
          state_ = TAKEOFF;
      }
    }
    else
      ROS_INFO("Waiting to take off.  Press Rec to turn on the motors and Play to Take off.");
  }
  else
  {
    // This is executed every time the midi controller changes
    switch(state_)
    {
      case VELOCITY_TRACKER:
        {
          double x = msg->axes[0] * fabs(msg->axes[0]) / 2;
          double y = msg->axes[1] * fabs(msg->axes[1]) / 2;
          double z = msg->axes[2] * fabs(msg->axes[2]) / 2;
          double yaw = msg->axes[3] * fabs(msg->axes[3]) / 2;

          //mav->setDesVelWorld(x, y, z, yaw);
          //ROS_INFO("Velocity Command: (%1.4f, %1.4f, %1.4f, %1.4f)", x, y, z, yaw);
        }
        break;

      default:
        break;
    }

    // Hover
    if(msg->buttons[hover_button])
    {
      // Marker Set
      if (mav->hover())
        state_ = HOVER;
    }
    else if(msg->buttons[home_button])
    {
      state_ = HOME;
      mav->goHome();
    }
    else if(msg->buttons[land_button])
    {
      state_ = LAND;
      mav->land();
    }
    // Line Tracker Yaw
    else if(msg->buttons[line_tracker_yaw_button])
    {
      double x = 2*msg->axes[0] + xoff;
      double y = 2*msg->axes[1] + yoff;
      double z = 2*msg->axes[2] + 2.0 + zoff;
      double yaw = M_PI * msg->axes[3] + yaw_off;

      if (mav->goTo(x, y, z, yaw))
        state_ = LINE_TRACKER;
    }
    // Velocity Tracker
    else if(msg->buttons[velocity_tracker_button] && state_ == HOVER)
    {
      // Note: We do not want to send a goal of 0 if we are
      // already in the velocity tracker controller since it
      // could cause steps in the velocity.

      ROS_INFO("Engaging controller: VELOCITY_TRACKER");
      //if (mav->setDesVelWorld(0,0,0))
        //state_ = VELOCITY_TRACKER;
    }
    else if(msg->buttons[traj_button])
    {
      // If there are any errors
      if (traj.isLoaded())
      {
        traj.set_start_time();
        traj.UpdateGoal(traj_goal);

        double x = traj_goal.position.x;
        double y = traj_goal.position.y;
        double z = traj_goal.position.z;
        double yaw = traj_goal.yaw;

        if (mav->goTo(x, y, z, yaw))
          state_ = PREP_TRAJ;
      }      
    }
  }
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{

  if (state_ != TRAJ)
  {
      odom_des.header.stamp = ros::Time::now();
      odom_des.pose.pose.position.x = msg->pose.pose.position.x;
      odom_des.pose.pose.position.y = msg->pose.pose.position.y;
      odom_des.pose.pose.position.z = msg->pose.pose.position.z;
      odom_des.pose.pose.orientation.w = 1;//save just the yaw in the scalar part of the quaternion
      odom_des.pose.pose.orientation.x = 0;
      odom_des.pose.pose.orientation.y = 0;
      odom_des.pose.pose.orientation.z = 0;
      odom_des.twist.twist.linear.x = msg->twist.twist.linear.x;
      odom_des.twist.twist.linear.y = msg->twist.twist.linear.x;
      odom_des.twist.twist.linear.z = msg->twist.twist.linear.x;
      des_odom_pub.publish(odom_des);
 }
  // If we are currently executing a trajectory, update the setpoint
  if (state_ == TRAJ)
  {
    if (traj.isCompleted())
    {
      // Publish the trajectory signal
      std_msgs::Bool traj_on_signal;
      traj_on_signal.data = false;
      pub_traj_signal_.publish(traj_on_signal);

      // Reset the trajectory and start again
      traj.set_start_time();
      traj.UpdateGoal(traj_goal);

      double x = traj_goal.position.x;
      double y = traj_goal.position.y;
      double z = traj_goal.position.z;
      double yaw = traj_goal.yaw;
      std::cout << GREEN << "Completed trajectory " <<std::endl;
   if(traj_num_ > max_traj_num){
        if(mav->hover())
        state_ = HOVER;
	traj_num_ = 0;
	}
     // else if (mav->goTo(x, y, z, yaw))
       // state_ = PREP_TRAJ;
    }
    else
    {
      odom_des.header.stamp = ros::Time::now();
      traj.UpdateGoal(traj_goal);
      mav->setPositionCommand(traj_goal);

      std_msgs::Int16 traj_num_msg;
      traj_num_msg.data = traj_num_;
      pub_traj_num_.publish(traj_num_msg);
      //publish the desired trajectory
      
      odom_des.pose.pose.position.x = traj_goal.position.x;
      odom_des.pose.pose.position.y = traj_goal.position.y;
      odom_des.pose.pose.position.z = traj_goal.position.z;
      odom_des.pose.pose.orientation.w = traj_goal.yaw;//save just the yaw in the scalar part of the quaternion
      odom_des.pose.pose.orientation.x = 0;
      odom_des.pose.pose.orientation.y = 0;
      odom_des.pose.pose.orientation.z = 0;
      odom_des.twist.twist.linear.x = traj_goal.velocity.x;
      odom_des.twist.twist.linear.y = traj_goal.velocity.y;
      odom_des.twist.twist.linear.z = traj_goal.velocity.z;
      des_odom_pub.publish(odom_des);
    }
  }

  if(state_ == PREP_TRAJ)
  {
    // Updates traj goal to allow for correct initalization of the trajectory
    traj.set_start_time();
          odom_des.header.stamp = ros::Time::now();

    traj.UpdateGoal(traj_goal);
    // If we are ready to start the trajectory
    Eigen::Vector3f pos = mav->pos();
    Eigen::Vector3f vel = mav->vel();

    if ( sqrt( pow(traj_goal.position.x - pos[0], 2)
             + pow(traj_goal.position.y - pos[1], 2)
             + pow(traj_goal.position.z - pos[2], 2) ) < 0.1 &&
         sqrt( pow(vel[0],2) + pow(vel[1],2) + pow(vel[2],2) ) < 0.1)
    {
      
      state_ = TRAJ;
      traj_num_++;

      std::cout << GREEN << "Starting trajectory " << traj_num_ << RESET << std::endl;

      // Publish the trajectory signal
      std_msgs::Bool traj_on_signal;
      traj_on_signal.data = true;
      pub_traj_signal_.publish(traj_on_signal);

      traj.set_start_time();
      traj.UpdateGoal(traj_goal);
      mav->setPositionCommand(traj_goal);
    }
    else
    {
      ROS_WARN_THROTTLE(2, "Not ready to start trajectory.");
      ROS_INFO_THROTTLE(2,
        RED "rdes - r = {%2.2f, %2.2f, %2.2f}, vel = %2.2f" RESET,
        traj_goal.position.x - pos[0],
        traj_goal.position.y - pos[1],
        traj_goal.position.z - pos[2],
        sqrt(pow(vel[0],2) + pow(vel[1],2) + pow(vel[2],2)));
    }
  }
}

double norm(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
  return std::sqrt(
      std::pow(a.x - b.x, 2)
    + std::pow(a.y - b.y, 2)
    + std::pow(a.z - b.z, 2));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_control");
  ros::NodeHandle n("~");

  // Position offsets for this robot
  n.param("offsets/x", xoff, 0.0);
  n.param("offsets/y", yoff, 0.0);
  n.param("offsets/z", zoff, 0.0);
  n.param("offsets/yaw", yaw_off, 0.0);
  ROS_INFO("Using offsets: {xoff: %2.2f, yoff: %2.2f, zoff: %2.2f, yaw_off: %2.2f}", xoff, yoff, zoff, yaw_off);

  n.param("traj_filename", traj_filename, string(""));

  traj.set_filename(traj_filename.c_str());
  traj.setOffsets(xoff, yoff, zoff, yaw_off);
  traj.LoadTrajectory();
  if (traj.isLoaded())
    ROS_INFO("Successfully loaded trajectory: %s.", traj_filename.c_str());
  else
  {
    ROS_ERROR("Error Code: %d. Could not load %s", traj.get_error_code(), traj_filename.c_str());
    return 1;
  }


  // Publishers
  pub_traj_signal_ = n.advertise<std_msgs::Bool>("traj_signal", 1);
  pub_traj_num_ = n.advertise<std_msgs::Int16>("traj_num", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());
  // ros::Subscriber sub_imu = n.subscribe("quad_decode_msg/imu", 1, &imu_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 1, nanokontrol_cb, ros::TransportHints().tcpNoDelay());
  des_odom_pub = n.advertise<nav_msgs::Odometry>("des_odom", 1);
  // MAVManager stuff
  mav.reset(new MAVManager());
  mav->set_need_imu(false);
  mav->set_use_attitude_safety_catch(false);

  ros::spin();

  return 0;
}
