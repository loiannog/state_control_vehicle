#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Geometry>
#include <nav_msgs/Path.h>


#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <iostream>
using namespace std;


// global variables
visualization_msgs::Marker linemark;
visualization_msgs::Marker box_name;
#define PI 3.14159265359

double r, g, b, a;
Eigen::Matrix<double,1,4> locations;
double yaw;
double roll;
double pitch;
vector<string> locations_names;
double dim_box_x;
double dim_box_y;
double dim_box_z;
double x;
double y;
double z;

void create_box_area();
Eigen::Matrix<double, 3, 1> ypr;
Eigen::Matrix<double, 3, 1> p2;

Eigen::Matrix<double, 3, 3> ypr_to_R(Eigen::Matrix<double, 3, 1> ypr){

  Eigen::Matrix<double, 3, 3> Rz;
  double y = ypr(0);
  double c = cos(y);
  double s = sin(y);
  Rz(0,0) =  c;
  Rz(1,0) =  s;
  Rz(0,1) = -s;
  Rz(1,1) =  c;
  Rz(2,2) =  1;

  Eigen::Matrix<double, 3, 3> Ry;
  double p = ypr(1);
  c = cos(p);
  s = sin(p);
  Ry(0,0) =  c;
  Ry(2,0) = -s;
  Ry(0,2) =  s;
  Ry(2,2) =  c;
  Ry(1,1) =  1;

  Eigen::Matrix<double, 3, 3> Rx;
  double r = ypr(2);
  c = cos(r);
  s = sin(r);
  Rx(1,1) =  c;
  Rx(2,1) =  s;
  Rx(1,2) = -s;
  Rx(2,2) =  c;
  Rx(0,0) =  1;

  Eigen::Matrix<double, 3, 3> R = Rz*Ry*Rx;
  return R;
}

Eigen::Matrix<double, 3, 1> R_to_ypr(Eigen::Matrix<double, 3, 3>& R){

  Eigen::Matrix<double, 3, 1> n = R.block(0,0,3,1);
  Eigen::Matrix<double, 3, 1> o = R.block(0,1,3,1);
  Eigen::Matrix<double, 3, 1> a = R.block(0,2,3,1);
  
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0)*cos(y)+n(2)*sin(y));
  double r = atan2(a(0)*sin(y)-a(1)*cos(y), -o(0)*sin(y)+o(1)*cos(y));
  Eigen::Matrix<double, 3, 1> ypr;
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr;

}

Eigen::Matrix<double, 3,3> QuatToMat(const Eigen::Matrix<double, 4,1>& Quat){
  Eigen::Matrix<double, 3,3> Rot;
  double s = Quat(0,0);
  double x = Quat(1,0);
  double y = Quat(2,0);
  double z = Quat(3,0);
  Rot<< 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
          2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
          2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);

  return Rot;
}

void check_odom(const nav_msgs::Odometry& odom){
Eigen::Matrix<double, 4,1> Quat_odom;
Quat_odom(1) = odom.pose.pose.orientation.x;
Quat_odom(2) = odom.pose.pose.orientation.y;
Quat_odom(3) = odom.pose.pose.orientation.z;
Quat_odom(0) = odom.pose.pose.orientation.w;

Eigen::Matrix<double, 3,3> R = QuatToMat(Quat_odom);
Eigen::Matrix<double, 3,1> ypr = R_to_ypr(R);
//cout<<"roll:"<<ypr(2) * 180/3.14<<endl;

}


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "window_visualizer_env");
	ros::NodeHandle n("~");
	//ros::Publisher landingmarkadv = n.advertise<visualization_msgs::MarkerArray>("vslam/landingmarks", 1);
	//ros::Publisher landingmarkadv_id = n.advertise<visualization_msgs::MarkerArray>("vslam/landingmarks_id", 1);
	ros::Publisher boxadv = n.advertise<visualization_msgs::Marker>("vslam/boxarea", 1);
	ros::Publisher boxnameadv = n.advertise<visualization_msgs::Marker>("vslam/boxname", 1);
    //ros::Publisher sub_goaladv = n.advertise<visualization_msgs::Marker>("vslam/marksassociationgoals", 1);
    //startslocationsadv = n.advertise<visualization_msgs::MarkerArray>("vslam/starts_locations", 1);
    ros::Subscriber sub_odom = n.subscribe("/quadrotor/odom", 10, &check_odom);
    
    n.param<double>("color/r", r, 1.0);
    n.param<double>("color/g", g, 0.0);
    n.param<double>("color/b", b, 0.0);
    n.param<double>("color/a", a, 0.6);
    n.param<double>("dim_box_x", dim_box_x, 6.0);
    n.param<double>("dim_box_y", dim_box_y, 6.0);
    n.param<double>("dim_box_z", dim_box_z, 3.0);
    n.param<double>("x", x, 6.0);
    n.param<double>("y", y, 3.0);
    n.param<double>("z", z, 3.0);
    n.param<double>("yaw", yaw, 0.0);
    n.param<double>("roll", roll, 0.0);
    n.param<double>("pitch", pitch, 0.0);
    p2(0) = x;
    p2(1) = y;
    p2(2) = z;
    ypr(0) = yaw*PI/180;
    ypr(1) = pitch*PI/180;
    ypr(2) = roll*PI/180;


	create_box_area();

	while(ros::ok())
	{
		//landingmarkadv.publish(landingmark_tot);
		//landingmarkadv_id.publish(Ids3d);
		boxadv.publish(linemark);
		boxnameadv.publish(box_name);
        //sub_goaladv.publish(goals_lines);
        ros::spinOnce();
		usleep(1000);
	}
	return 0;
}


void create_box_area(){
	linemark.id = 0;
	linemark.lifetime = ros::Duration(1);
	linemark.header.frame_id = "simulator";
	linemark.action = visualization_msgs::Marker::ADD;
	linemark.type = visualization_msgs::Marker::LINE_LIST;
	linemark.ns = "linelist_publisher";
	linemark.color.r = 0;
	linemark.color.g = 1;
	linemark.color.b = 1;
	linemark.color.a = 1;
	linemark.scale.x = 0.1;
    //define the window points
    Eigen::Matrix<double, 3, 3> R_temp;
    R_temp <<  0, 0, 1,
               0, 1, 0,
               1, 0, 0;
    Eigen::Matrix<double, 3, 3> R_window = ypr_to_R(ypr);
    Eigen::Matrix<double, 3, 1> p1_window_temp, p2_window_temp, p3_window_temp, p4_window_temp;
    p1_window_temp << dim_box_y/2, dim_box_x/2, 0;
    p2_window_temp << -dim_box_y/2, dim_box_x/2, 0; 
    p3_window_temp << dim_box_y/2, -dim_box_x/2, 0; 
    p4_window_temp << -dim_box_y/2, -dim_box_x/2, 0; 

    Eigen::Matrix<double, 3, 1> p1_window = p2 + R_window*R_temp*p1_window_temp;
    Eigen::Matrix<double, 3, 1> p2_window = p2 + R_window*R_temp*p2_window_temp;
    Eigen::Matrix<double, 3, 1> p3_window = p2 + R_window*R_temp*p3_window_temp;
    Eigen::Matrix<double, 3, 1> p4_window = p2 + R_window*R_temp*p4_window_temp;

	//create a vector of points
	geometry_msgs::Point p1a,p2a,p3a,p4a;
	geometry_msgs::Point p1b,p2b,p3b,p4b;
	double shift_x = dim_box_x/2;
    p1a.x = p1_window(0);
    p1a.y = p1_window(1);
    p1a.z = p1_window(2);
    p2a.x = p2_window(0);
    p2a.y = p2_window(1);
    p2a.z = p2_window(2);

    p3a.x = p3_window(0);
    p3a.y = p3_window(1);
    p3a.z = p3_window(2);

    p4a.x = p4_window(0);
    p4a.y = p4_window(1);
    p4a.z = p4_window(2);
    linemark.points.push_back(p1a);
    linemark.points.push_back(p3a);
    linemark.points.push_back(p3a);
    linemark.points.push_back(p4a);
    linemark.points.push_back(p4a);
    linemark.points.push_back(p2a);
    linemark.points.push_back(p2a);
    linemark.points.push_back(p1a);

    //put a text for the flying area
    box_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    box_name.pose.position.x = 4.5;//shift_x;
    box_name.pose.position.y = 0;
    box_name.pose.position.z = 0.0;
    box_name.id = 0;
    box_name.header.frame_id = "simulator";
    box_name.color.a = 1.0;
    box_name.scale.z = 1.0;
    box_name.color.r = 1.0;
    box_name.color.b = 0.0;
    box_name.text = "Window";//"CES Flying Arena";
}


