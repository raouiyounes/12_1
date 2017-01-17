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

void image_callback(const sensor_msgs::ImageConstPtr& image) {
		 Mat image_init=imread("/home/younes/Images/test.jpg");
	DMatch* good_matches;
	good_matches=(DMatch*)malloc(sizeof(DMatch)*FPVision::k);

	try
	  	    {

		if(nbre_of_captures++==0){
	 		 cv_bridge::CvImagePtr   cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
			Mat image_init=cv_ptr->image;
			image_history.push_back(image_init);
	  		landmarks=fp->compute_initial_pose_land(image_init);
		}
	  		cv_bridge::CvImagePtr   cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	  		image_history.push_back(cv_ptr->image);
	  		landmarks=fp->compute_initial_pose_land(image_init);

fp->predict_pose_landmarks(2.2,3.3,2.2);

//cout<<fp->feature_point_predict[0].at(0).at(0)<<"kkkkkkkkkkkkkkkkkkkkkk";

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

	  boost::property_tree::ptree settings, general_settings;
	  read_ini(argv[1], settings);
	  ratslam::get_setting_child(general_settings, settings, "general", true);
	    ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

	    if (!ros::isInitialized())
	    {
	      ros::init(argc, argv, "RatSLAMVisualOdometry");
	    }

	  ros::NodeHandle node;

	   image_transport::ImageTransport it(node);
	  image_transport::Subscriber sub = it.subscribe(topic_root+"/camera/image", 1, image_callback);
	  ros::spin();
  delete fp;

}


