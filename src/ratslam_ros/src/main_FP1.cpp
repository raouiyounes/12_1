#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ratslam/FPVision.h"
#include "utils/utils.h"
#include "ratslam/visual_odometry.h"
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


void odo_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

cout<<"eeeeeeeeeeeeeee";
}

void image_callback(const sensor_msgs::ImageConstPtr& image) {

		 Mat image_init=imread("/home/younes/Images/test.jpg");
	DMatch* good_matches;
	good_matches=(DMatch*)malloc(sizeof(DMatch)*FPVision::k);



	  static nav_msgs::Odometry odom_output;

	try
	  	    {
		  pub_vo.publish(odom_output);

				cout<<"e	ee";
				cout<<odom_output.twist.twist.linear.y;
				  odom_output.header.seq++;

		if(nbre_of_captures++==0){
	 		 cv_bridge::CvImagePtr   cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
			Mat image_init=cv_ptr->image;
			image_history.push_back(image_init);
	  		landmarks=fp->compute_initial_pose_land(image_init);
		}
	  		cv_bridge::CvImagePtr   cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	  		image_history.push_back(cv_ptr->image);
	  		landmarks=fp->compute_initial_pose_land(image_init);

//fp->predict_pose_landmarks(2.2,3.3,2.2);

cout<<"kkkkkkkkkkkkkkkkkkkkkk"<<fp->k++<<"  "<<endl;

	  		//good_matches=fp->surf_extractor(cv_ptr->image,previous_image);
	  		previous_image=cv_ptr->image;
	  		nbre_of_captures++;
	  	    }
		catch (cv_bridge::Exception& e)
		    {
		      ROS_ERROR("cv_bridge exception: %s", e.what());
		      return;
		    }
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


	  if (!ros::isInitialized())
	  {
	    ros::init(argc, argv, "image_listener55",ros::init_options::AnonymousName);
	  }
	  ros::NodeHandle node;
	  ros::NodeHandle node1;

	   image_transport::ImageTransport it(node);


	  pub_vo = node1.advertise<nav_msgs::Odometry>(topic_root + "/odom", 0);
	  ros::Subscriber sub_odometry = node1.subscribe<nav_msgs::Odometry>("/stlucia/odom", 1000, odo_callback);

	 image_transport::Subscriber sub = it.subscribe(topic_root+"/camera/image", 1000, image_callback);
	  ros::spin();
  delete fp;
return 0;
}


