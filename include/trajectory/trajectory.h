#include <string>
#include <vector>

typedef std::vector< std::vector< std::vector<double> > > traj_type;

// int loadTraj(const std::string &filename, traj_type &traj);
ros::Time getMonotonicTime();
ros::Time getRealTimeRos();

class Trajectory
{
  private:
    int error_code_;
    std::string filename_;
    ros::Time start_time_;
    bool completed, loaded;
    double xoff, yoff, zoff, yaw_off;


  public:
    Trajectory();
    bool LoadTrajectory();
    void UpdateGoal(quadrotor_msgs::PositionCommand&);

    void set_start_time() {start_time_ = getRealTimeRos(); completed = false;}//ros::Time::now(); }
    void set_start_time(ros::Time time) {start_time_ = time; completed = false;}
    void set_filename(const std::string fname) {filename_ = fname;}
    void setOffsets(double, double, double, double);
    traj_type traj_;

    traj_type get_traj() {return traj_;}
    int  get_error_code() {return error_code_;}
    bool isLoaded() {return loaded;}
    bool isCompleted() {return completed;}
    std::string get_filename() {return filename_;}
};
