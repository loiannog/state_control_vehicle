#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <trajectory/trajectory.h>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

Trajectory::Trajectory() : completed(false), loaded(false), xoff(0), yoff(0), zoff(0), yaw_off(0) {}

void Trajectory::setOffsets(double x, double y, double z, double yaw) {
  xoff = x; yoff = y; zoff = z; yaw_off = yaw;
}

void Trajectory::UpdateGoal(quadrotor_msgs::PositionCommand &goal)
{
  ros::Duration delta_time = ros::Time::now() - start_time_;
  double traj_time = delta_time.toSec();

  unsigned long i = traj_time * 1000;

  if (i > traj_.size()-1)
  {
    i = traj_.size()-1;

    if (!completed)
    {
      ROS_INFO("Trajectory completed.");
      completed = true;
    }
  }

  goal.position.x = traj_[i][0][0] + xoff;
  goal.position.y = traj_[i][1][0] + yoff;
  goal.position.z = traj_[i][2][0] + zoff;

  goal.velocity.x = traj_[i][0][1];
  goal.velocity.y = traj_[i][1][1];
  goal.velocity.z = traj_[i][2][1];

  goal.acceleration.x = traj_[i][0][2];
  goal.acceleration.y = traj_[i][1][2];
  goal.acceleration.z = traj_[i][2][2];

  goal.jerk.x = traj_[i][0][3];
  goal.jerk.y = traj_[i][1][3];
  goal.jerk.z = traj_[i][2][3];

  goal.yaw = traj_[i][3][0] + yaw_off;
  goal.yaw_dot = traj_[i][3][1];

  goal.kx[0] = traj_[i][4][0];
  goal.kx[1] = traj_[i][4][1];
  goal.kx[2] = traj_[i][4][2];
  goal.kv[0] = traj_[i][4][3];
  goal.kv[1] = traj_[i][4][4];
  goal.kv[2] = traj_[i][4][5];
}

/*
int main()
{
  // traj[time_idx][flat_output][derivative]
  vector< vector< vector<double> > > traj;

  int success = loadTraj("traj.csv", traj);

  cout << "Returned: " << success << endl;

  // Output the array

  // Loop through flat outputs
  for (unsigned int i2=0; i2 < traj[0].size(); i2++)
  {
    cout << "Flat Output (group): " << i2 << endl;

    // Loop through times
    for (unsigned int i1=0; i1 < traj.size(); i1++)
    {

      // Loop through derivatives
      for (unsigned int i3=0; i3 < traj[i1][i2].size(); i3++)
      {
        cout << traj[i1][i2][i3] << " ";
      }
      cout << endl;
    }
    cout << endl;
  }

  for (unsigned int i3=0; i3 < traj[0][0].size(); i3++)
  {
    cout << "Derivative: " << i3 << endl;
    for (unsigned int i1=0; i1 < traj.size(); i1++)
    {
      // Flat Outputs
      for (unsigned int i2=0; i2 < traj[0].size(); i2++)
      {
        cout << traj[i1][i2][i3] << " ";
      }
      cout << endl;
    }
    cout << endl;
  }
}
*/

bool Trajectory::LoadTrajectory()
{
  // Much of this is from http://www.cplusplus.com/forum/unices/112048/

  // dims[0] is the number of rows
  // dims[1] is the number of flat outputs
  // dims[2] is the number of derivatives
  // dims[3] is the number of additional columns that include things like gains, servo values, etc.
  int dims[4];

  // Load the file
  std::ifstream file(filename_.c_str());

  // Define some variables
  std::string line;
  std::string val;

  // Read the first line and make sure it is good
  std::getline(file, line);
  if (!file.good())
  {
    error_code_ = 1;
    loaded = false;
    return false;
  }

  // Parse the line for commas
  // Note: we expect that the first line contains
  // # of time steps, # of flat outputs, # of derivatives
  // such that the product of the first line is the total number of elements
  std::stringstream iss(line);
  for(int idx=0; idx<4; idx++)
  {
    std::getline(iss, val, ',');
    if (iss.fail())
    {
      error_code_ = 2;
      loaded = false;
      return false;
    }

    std::stringstream convertor(val);
    convertor >> dims[idx];
  }

  cout << "Dimensions: {" << dims[0] << ", " << dims[1] << ", " << dims[2] << "}" << " + " << dims[3] << " additional columns" << endl;

  // Resize the trajectory vector of vectors
  //
  // Note: traj_[t_idx][flat_out][deriv]
  //
  traj_.resize(dims[0]);
  for (int i = 0; i < dims[0]; i++)
  {
    traj_[i].resize(dims[1]+1);
    for (int j = 0; j < dims[1]; j++)
    {
      traj_[i][j].resize(dims[2]);
    }
    traj_[i][dims[1]].resize(dims[3]);
  }

  // This dimension indexes the time
  for(int dim0 = 0; dim0 < dims[0]; dim0++)
  {
    // Get the entire line
    std::getline(file, line);
    if (file.fail())
    {
      cout << "Error reading line " << dim0+1 << endl;
      error_code_ = 3;
      loaded = false;
      return false;
    }

    // Create a stringstream for the line
    std::stringstream iss(line);

    // This dimension is the derivative
    for (int dim2 = 0; dim2 < dims[2]; dim2++)
    {
      // This dimension is the flat output
      for (int dim1 = 0; dim1 < dims[1]; dim1++)
      {
        std::getline(iss, val, ',');
        if (iss.fail())
        {
          error_code_ = 4;
          loaded = false;
          return false;
        }

        // The converter
        std::stringstream convertor(val);
        convertor >> traj_[dim0][dim1][dim2];
      }
    }

    // The additional columns
    for (int dim3 = 0; dim3 < dims[3]; dim3++)
    {
      std::getline(iss, val, ',');
      if (iss.fail())
      {
        error_code_ = 5;
        loaded = false;
        return false;
      }

      // The converter
      std::stringstream convertor(val);
      convertor >> traj_[dim0][dims[1]][dim3];
    }
  }

  cout << "Trajectory loaded" << endl;
  error_code_ = 0;
  loaded = true;
  return true;
}
