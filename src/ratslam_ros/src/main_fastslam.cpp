#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <ratslam_ros/poses_robot.h>
#include <sstream>
#include <ctime>
#include <random>
#include "/home/younes/catkino/src/ratslam_ros/src/ratslam/include_fs/loMeasurement.h"
#include "/home/younes/catkino/src/ratslam_ros/src/ratslam/include_fs/particle.h"
#include "/home/younes/catkino/src/ratslam_ros/src/ratslam/include_fs/sitRobotMM.h"
#include <Eigen/Dense>
#include <iostream>
#include "ratslam/Filter.h"
#include <ratslam_ros/toute.h>

using namespace std;

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include "ratslam/posecell_network.h"
#include <ratslam_ros/TopologicalAction.h>
#include <nav_msgs/Odometry.h>
#include <ratslam_ros/ViewTemplate.h>
#define numberOfParticles 50
#define pi 3.14
#if HAVE_IRRLICHT
#include "graphics/posecell_scene.h"
ratslam::PosecellScene *pcs;
bool use_graphics;
#endif

using namespace ratslam;
using namespace Eigen;
ratslam_ros::TopologicalAction pc_output;

#define numParticles 100

loMeasurement* currentMeasurement;
particle* particles[numParticles];

clock_t startTime = 0, endTime = 0;


void chatterCallback(ratslam_ros::toute::ConstPtr  msg)
{


	cout<< msg->landX[0]<<endl;
	  ROS_INFO("Idddd heard: [%f]", msg->landX[0]);

}
void odomCallback(ratslam_ros::poses_robot::ConstPtr msg) {


	cout<<msg->x_robot<<endl;
	/*
	currentMeasurement->updatePosition(odom);

	double xav = 0, yav = 0, theta = 0;
	for (int i = 0; i < numParticles; i++) {
		particles[i]->doUpdate(currentMeasurement);
		xav += particles[i]->getX();
		yav += particles[i]->getY();
		theta += particles[i]->getTheta();
	}
	xav /= numParticles;
	yav /= numParticles;
	theta /= numParticles;

	ROS_INFO(currentMeasurement->toString().c_str());
	ROS_INFO("Particle: (%f, %f, %f)", xav, yav, theta);
*/
}

float AngleWrap(float angle) {
	if (angle < -2 * pi)
		angle = angle + 2 * pi;
	else if (angle > 2 * pi)
		angle = angle - 2 * pi;
	return angle;

}
void poses(ratslam_ros::poses_robot::ConstPtr msg) {

	float angle = 0.0;
	std::normal_distribution<double> distribution(5.0, 2.0);
	std::random_device rd;
	std::mt19937 gen(rd());
	int i;
	float sample;
	float variance = 0.04;
	float a = 3.4, b = 3.4, c = pi;
	bool test;
	float xPartic, yPartic, thetaPartic;
	MatrixXd particle(3, numberOfParticles);
	angle = AngleWrap(msg->theta_robot);
	std::normal_distribution<float> dx(msg->x_robot, variance);
	std::normal_distribution<float> dy(msg->y_robot, variance);
	std::normal_distribution<float> dtheta(angle, variance);
	std::vector<float> centerOfCloud(3);
	centerOfCloud= {msg->x_robot,msg->y_robot,angle};
	for (int i = 0; i < numberOfParticles; i++) {
		xPartic = dx(gen);
		yPartic = dy(gen);
		thetaPartic = dtheta(gen);
		if( pow(xPartic-centerOfCloud.at(0),2)/(a*a)+pow(yPartic-centerOfCloud.at(1),2)/(b*b)+pow(thetaPartic-centerOfCloud.at(2),2)/(c*c)<1)
		test=true;
		else
			test=false;
		if (test == true) {
			particle(0, i) = xPartic;
			particle(1, i) = yPartic;
			particle(2, i) = thetaPartic;
		}
	}
}
void tfCallback(boost::shared_ptr<const nav_msgs::Odometry> tf) {
	ROS_INFO("Particle: (%f, %f)", tf->pose.pose.position.x,
			tf->pose.pose.position.y);
}

void odo_callback(nav_msgs::OdometryConstPtr odo, ratslam::PosecellNetwork *pc,
		ros::Publisher * pub_pc) {
	cout << odo->twist.twist.angular.z;
	/*  ROS_DEBUG_STREAM("PC:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);

	 static ros::Time prev_time(0);

	 if (prev_time.toSec() > 0)
	 {
	 double time_diff = (odo->header.stamp - prev_time).toSec();

	 pc_output.src_id = pc->get_current_exp_id();
	 pc->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
	 pc_output.action = pc->get_action();
	 if (pc_output.action != ratslam::PosecellNetwork::NO_ACTION)
	 {
	 pc_output.header.stamp = ros::Time::now();
	 pc_output.header.seq++;
	 pc_output.dest_id = pc->get_current_exp_id();
	 pc_output.relative_rad = pc->get_relative_rad();
	 pub_pc->publish(pc_output);
	 ROS_DEBUG_STREAM("PC:action_publish{odo}{" << ros::Time::now() << "} action{" << pc_output.header.seq << "}=" <<  pc_output.action << " src=" << pc_output.src_id << " dest=" << pc_output.dest_id);
	 }

	 #ifdef HAVE_IRRLICHT
	 if (use_graphics)
	 {
	 pcs->update_scene();
	 pcs->draw_all();
	 }
	 #endif
	 }
	 prev_time = odo->header.stamp;
	 */
}

void template_callback(ratslam_ros::ViewTemplateConstPtr vt,
		ratslam::PosecellNetwork *pc, ros::Publisher * pub_pc) {
	ROS_DEBUG_STREAM(
			"PC:vt_callback{" << ros::Time::now() << "} seq=" << vt->header.seq
			<< " id=" << vt->current_id << " rad=" << vt->relative_rad);
	pc->on_view_template(vt->current_id, vt->relative_rad);
#ifdef HAVE_IRRLICHT
	if (use_graphics)
	{
		pcs->update_scene();
		pcs->draw_all();
	}
#endif
}

int main(int argc, char * argv[]) {
	//ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
	//ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
	//ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");
	if (argc < 2) {
		ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
		exit(-1);
	}
	std::string topic_root = "";
	boost::property_tree::ptree settings, ratslam_settings, general_settings;
	read_ini(argv[1], settings);
	get_setting_child(ratslam_settings, settings, "ratslam", true);
	get_setting_child(general_settings, settings, "general", true);
	get_setting_from_ptree(topic_root, general_settings, "topic_root",
			(std::string) "");
	currentMeasurement = new loMeasurement();
	ros::init(argc, argv, "fastSLAM");
	ros::NodeHandle n;
	// Create Particles
	movementModel* mm = new sitRobotMM();
	for (int i = 0; i < numParticles; i++) {
		particles[i] = new particle(-0.004, 0.0, 0.0);
		particles[i]->set_movement_model(mm);
	}

	if (!ros::isInitialized()) {
		ros::init(argc, argv, "odom_listener");
	}
	ros::NodeHandle node;

	ratslam::PosecellNetwork * pc = new ratslam::PosecellNetwork(
			ratslam_settings);
	ros::Publisher pub_pc = node.advertise < ratslam_ros::TopologicalAction
			> (topic_root + "/PoseCell/TopologicalAction", 0);

	//::Subscriber s = n.subscribe(topic_root + "/odom", 1000, odomCallback);

//	 ros::Subscriber sub_template = n.subscribe<ratslam_ros::poses_robot>("poses", 1,odomCallback);
	    ros::Subscriber sub_template = n.subscribe<ratslam_ros::toute>("landmark", 1000,chatterCallback);

	//ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, pc, &pub_pc), ros::VoidConstPtr(),
	//                                                                ros::TransportHints().tcpNoDelay());
	//ros::Subscriber sub_template = node.subscribe<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0, boost::bind(template_callback, _1, pc, &pub_pc),
	//                                                                       ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

	ros::spin();

	return 0;
}

