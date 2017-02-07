#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ratslam/FPVision.h"
#include "utils/utils.h"
#include "ratslam/visual_odometry.h"
#include "ratslam/posecell_network.h"
#include <boost/property_tree/ini_parser.hpp>
#include <cv_bridge/cv_bridge.h>
#include "fstream"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
using namespace ratslam;
using namespace std;
Mat image_init=imread("/home/younes/Images/test.jpg");
typedef vector<vector<float> > Matos;
FPVision *fp=new FPVision(image_init,20);
int nbre_of_captures=0;

// 20 : nbre of landmarks
int FPVision::k=0;
Mat previous_image=image_init;
std::vector<Mat>  image_history;
Matos landmarks;
ros::Publisher pub_vo;
geometry_msgs::Twist speed;
ofstream myfile;

vector<float> xRobot;
float x,y,th;
void odo_callback(const nav_msgs::Odometry::ConstPtr& msg, ratslam::PosecellNetwork *pc)
{ // ROS_DEBUG_STREAM("PC:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << msg->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);
	myfile.open("file.txt");
	static ros::Time prev_time(0);
	//  if (prev_time.toSec() > 0)
	  //{
	    double time_diff = (msg->header.stamp - prev_time).toSec();
  pc->on_odo(msg->twist.twist.linear.x, msg->twist.twist.angular.z, time_diff);
  xRobot.push_back(pc->x());
  xRobot.push_back(pc->y());
  xRobot.push_back(pc->th());
x=pc->x();
y=pc->y();
th=pc->th();
//}
}
void image_callback(const sensor_msgs::ImageConstPtr& image) {
		 Mat image_init=imread("/home/younes/Images/test.jpg");
	DMatch* good_matches;
	good_matches=(DMatch*)malloc(sizeof(DMatch)*FPVision::k);
	 try {
		if(nbre_of_captures++==0){
	 		 cv_bridge::CvImagePtr   cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
			Mat image_init=cv_ptr->image;
			image_history.push_back(image_init);
	  		landmarks=fp->compute_initial_pose_land(image_init);
		}
	  		cv_bridge::CvImagePtr   cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	  		image_history.push_back(cv_ptr->image);
	  		landmarks=fp->compute_initial_pose_land(image_init);
fp->predict_pose_landmarks(x,y,th);
fp->predict_feature_points();

cout<<"eeeee";
fp->surf_extractor(cv_ptr->image,cv_ptr->image);
	 		previous_image=cv_ptr->image;
//	  		nbre_of_captures++;
//	for( int i = 0; i < 2; i++ )
//	{ printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
	 }
		catch (cv_bridge::Exception& e)
		    {
		      ROS_ERROR("cv_bridge exception: %s", e.what());
		      return;
		    }
		xRobot.clear();
		free(good_matches);
}
int main(int argc, char **argv)
{
	 if (argc < 2) {
	    ROS_FATAL_STREAM("USAGE: " << argv[0] << "<config_file>");
	    exit(-1);
	  }
	  std::string topic_root = "";
	  boost::property_tree::ptree settings, ratslam_settings, general_settings;
	  read_ini(argv[1], settings);
	  get_setting_child(ratslam_settings, settings, "ratslam", true);
	  get_setting_child(general_settings, settings, "general", true);
	  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
	  ratslam::PosecellNetwork * pc = new ratslam::PosecellNetwork(ratslam_settings);
	  if (!ros::isInitialized())
    ros::init(argc, argv, "test");
	  ros::NodeHandle node1;
	   image_transport::ImageTransport it(node1);
	   ros::Subscriber sub_odometry = node1.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 1, boost::bind(odo_callback, _1, pc), ros::VoidConstPtr(),
	                                                                     ros::TransportHints().tcpNoDelay());
	//   ros::Subscriber sub = node1.subscribe(topic_root +"/odom", 1, odo_callback);
	 image_transport::Subscriber sub1 = it.subscribe(topic_root+"/camera/image", 1, image_callback);
	  ros::spin();
  delete fp;
return 0;
}


