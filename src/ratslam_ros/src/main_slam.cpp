#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <ratslam_ros/poses_robot.h>
#include <sstream>
#include <ctime>
#include "ratslam/Filter.h"
#include <ratslam_ros/toute.h>
#include <iostream>
#include "utils/utils.h"
#include <boost/property_tree/ini_parser.hpp>
#include <ros/ros.h>
#include "ratslam/posecell_network.h"
#include <ratslam_ros/TopologicalAction.h>
#include <nav_msgs/Odometry.h>
#include <ratslam_ros/ViewTemplate.h>

using namespace ratslam;
using namespace std;
ratslam_ros::TopologicalAction pc_output;
#define numParticles 100
#define pi 3.14
#define numberOfParticles 50
float x,y,th;
Filter *filterRS=new Filter();


float AngleWrap(float angle) {
	if (angle < -2 * pi)
		angle = angle + 2 * pi;
	else if (angle > 2 * pi)
		angle = angle - 2 * pi;
	return angle;

}
void poses(const ratslam_ros::poses_robot::ConstPtr  msg) {
	int numbeOfLandmarks;
	std::map<int,vector<vector<float> >>  observationsFromCloud;


	filterRS->setNParti(numberOfParticles);
	float angle = 0.0;
	std::normal_distribution<double> distribution(5.0, 2.0);
	std::random_device rd;
	std::mt19937 gen(rd());
	int i;
	float sample;
	float variance = 0.04;
	float a = 16, b = 16, c = 50;
	bool test;
	float xPartic, yPartic, thetaPartic;
	MatrixXd particle(3, numberOfParticles);
	angle = AngleWrap(msg->theta_robot);
	std::normal_distribution<float> dx(msg->x_robot, variance);
	std::normal_distribution<float> dy(msg->y_robot, variance);
	std::normal_distribution<float> dtheta(angle, variance);
	std::vector<float> centerOfCloud(3);
	centerOfCloud.push_back(msg->x_robot);
	centerOfCloud.push_back(msg->y_robot);
	centerOfCloud.push_back(msg->theta_robot);
	for (int i = 0; i < numberOfParticles; i++) {

		xPartic = dx(gen);
			yPartic = dy(gen);
			thetaPartic = dtheta(gen);



	if( (pow(xPartic-centerOfCloud.at(0),2)/(a*a))+(pow(yPartic-centerOfCloud.at(1),2)/(b*b))+(pow(thetaPartic-centerOfCloud.at(2),2)/(c*c))<1)
		test=true;
		else
			test=false;


		if (test == true) {
			particle(0, i) = xPartic;
			particle(1, i) = yPartic;
			particle(2, i) = thetaPartic;


		}
	}
	MatrixXd robotPosePart;
	MatrixXd landmarks;
filterRS->setPoseCloud(particle);
observationsFromCloud=filterRS->observation();

}
void odo_callback(const nav_msgs::Odometry::ConstPtr& msg, ratslam::PosecellNetwork *pc)
{
	ros::NodeHandle node;
	ratslam_ros::poses_robot poses;
	ros::Publisher pubPose = node.advertise < ratslam_ros::poses_robot
				> ("poses", 1);
	static ros::Time prev_time(0);
	  if (prev_time.toSec() > 0)
	  {

	double time_diff = (msg->header.stamp - prev_time).toSec();
  pc->on_odo(msg->twist.twist.linear.x, msg->twist.twist.angular.z, time_diff);
  poses.x_robot=pc->x(); poses.y_robot=pc->y();poses.theta_robot=pc->th();
  pubPose.publish(poses);

}	  prev_time = msg->header.stamp;
}

void landmarksCbk( ratslam_ros::toute::ConstPtr msgLand){

MatrixXd landmarks(msgLand->land.size(),3);

for(int i=0;i<msgLand->land.size();i++){
	landmarks(i,0)=msgLand->land[i].valueX;
	landmarks(i,1)=msgLand->land[i].valueY;
	landmarks(i,2)=msgLand->land[i].valueZ;
}
filterRS->setNLand(msgLand->land.size());
filterRS->setLand(landmarks);

}
int main(int argc, char * argv[])
{
  //ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  //ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  //ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");
  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  std::string topic_root = "";
  boost::property_tree::ptree settings, ratslam_settings, general_settings;
  read_ini(argv[1], settings);
  get_setting_child(ratslam_settings, settings, "ratslam", true);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
	ros::init(argc, argv, "fastSLAM");
	ros::NodeHandle n;
	// Create Particles
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "odom_listener");
  }
  ros::NodeHandle node;

  ratslam::PosecellNetwork * pc = new ratslam::PosecellNetwork(ratslam_settings);
  //subscriber for image topic
 ros::Subscriber sub_templatel = n.subscribe<ratslam_ros::toute>("landmark", 1000,landmarksCbk);



  // subscriber for pose topic
 ros::Subscriber sub_template = n.subscribe<ratslam_ros::poses_robot>("poses", 1,poses);


  ros::spin();

  return 0;
}




