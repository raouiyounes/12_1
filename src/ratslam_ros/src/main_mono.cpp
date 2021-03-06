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
#include <ratslam_ros/toute.h>
#include <ratslam_ros/poses_robot.h>
#include <fstream>
#include <memory>
#include <iterator>
using namespace ratslam;
using namespace std;
Mat image_init = imread("/home/younes/Images/test.jpg");
typedef vector<vector<float> > Matos;

std::auto_ptr<int> ptr(new int);
FPVision *fp = new FPVision(image_init, 20);
int nbre_of_captures = 0;
// 20 : nbre of landmarks
int FPVision::k = 0;
Mat previous_image = image_init;
std::vector<Mat> image_history;
Matos landmarks;
ros::Publisher pub_vo;
geometry_msgs::Twist speed;
ofstream myfile;
Matos landmarksMeasure;
int sizeOfKeypoint;
vector<float> xRobot;
float x, y, th;
float rrr;

Matos data;
std::ofstream fileX("./landX.txt", ios::app);
std::ofstream fileY("./landY.txt", ios::app);
std::ofstream fileZ("./landZ.txt", ios::app);

void odo_callback(const nav_msgs::Odometry::ConstPtr& msg, ratslam::PosecellNetwork *pc)
{ // ROS_DEBUG_STREAM("PC:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << msg->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);

	myfile.open("file.txt");
	static ros::Time prev_time(0);
	//  if (prev_time.toSec() > 0)
	  //{

	std::ofstream poses("./poses.txt",std::ofstream::trunc);



	double time_diff = (msg->header.stamp - prev_time).toSec();
  pc->on_odo(msg->twist.twist.linear.x, msg->twist.twist.angular.z, time_diff);
  xRobot.push_back(pc->x());
  xRobot.push_back(pc->y());
  xRobot.push_back(pc->th());
x=pc->x();
y=pc->y();
th=pc->th();

poses<<x<<endl;
poses<<y<<endl;
poses<<th<<endl;

//}
}
void image_callback(const sensor_msgs::ImageConstPtr& image) {
	ros::NodeHandle node1;
	ros::Publisher pub = node1.advertise < ratslam_ros::toute
				> ("landmark", 1);



	rrr=890.0;

	*ptr = 36;
	std::vector < DMatch > matches;
	Mat image_init = imread("/home/younes/Images/test.jpg");
	try {
		if (nbre_of_captures++ == 0) {
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image,
					sensor_msgs::image_encodings::BGR8);
			Mat image_init = cv_ptr->image;
			image_history.push_back(image_init);
			landmarks = fp->compute_initial_pose_land(image_init);
		}
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image,
				sensor_msgs::image_encodings::BGR8);
		image_history.push_back(cv_ptr->image);
		fp->predict_pose_landmarks(x, y, th);
		fp->predict_feature_points();
		fp->keypoint = fp->surf_extractor(cv_ptr->image);
		matches = fp->surf_extractor(cv_ptr->image, previous_image);
		vector < KeyPoint > kp1 = fp->surf_extractor(cv_ptr->image);
		vector < KeyPoint > kp2 = fp->surf_extractor(previous_image);
		previous_image = cv_ptr->image;
// depth computation
		fp->measure_depth(kp1, kp2, matches);
		sizeOfKeypoint = kp1.size();
		data = fp->map_of_landmarksPredicted[fp->index_of_landmark];
		Matos landmark;
		vector<float> lineEntryLandmark;
		cout << "" << fp->index_of_landmark;
		landmarksMeasure = fp->landmarkXYandDepth(data);
		vector<float> landX, landY, landZ;
	fp->data1=data;
		for (int i = 0; i < fp->size_of_surf; i++) {
			landX.push_back(data.at(i).at(0));
			landY.push_back(data.at(i).at(1));
			landZ.push_back(data.at(i).at(2));
		}
		landmarksMeasure.clear();
		for (int i = 0; i < fp->size_of_surf; i++)
			fileX << landX.at(i) << endl;

		for (int i = 0; i < fp->size_of_surf; i++)
			fileY << landY.at(i) << endl;
		for (int i = 0; i < fp->size_of_surf; i++)
			fileZ << landZ.at(i) << endl;

	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}



	ratslam_ros::toute landmark;

	for(int i=0;i<10;i++){
				landmark.landX.push_back(data.at(i).at(0));
						landmark.landY.push_back(11.11);
						landmark.landZ.push_back(11.11);
	}
						pub.publish(landmark);


	xRobot.clear();

}


int analizeFile (ifstream &fin, const string &fileName) {
   int count = 1;
   int num;
   fin.open(fileName.c_str() );
   fin >> num;
   while (fin.good() ) {
      fin>> num;
      count ++;
   }
   return count;
}



int main(int argc, char **argv) {
	if (argc < 2) {
		ROS_FATAL_STREAM("USAGE: " << argv[0] << "<config_file>");
		exit(-1);
	}
	std::string topic_root = "";
	boost::property_tree::ptree settings, ratslam_settings, general_settings;
	read_ini(argv[1], settings);
	get_setting_child(ratslam_settings, settings, "ratslam", true);
	get_setting_child(general_settings, settings, "general", true);
	get_setting_from_ptree(topic_root, general_settings, "topic_root",
			(std::string) "");
	ratslam::PosecellNetwork * pc = new ratslam::PosecellNetwork(
			ratslam_settings);
	if (!ros::isInitialized())
		ros::init(argc, argv, "test");
	ros::NodeHandle node1, node2,node3;
	image_transport::ImageTransport it(node2);
	ros::Subscriber sub_odometry =
			node1.subscribe < nav_msgs::Odometry
					> (topic_root + "/odom", 1, boost::bind(odo_callback, _1,
							pc), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());



//	ratslam_ros::scalarMess val;
	ratslam_ros::toute landmark;
	ratslam_ros::poses_robot poses;
	int k = 0;
	vector<float> lineLandmark;
	ros::Publisher pub = node2.advertise < ratslam_ros::toute
			> ("landmark", 1);

ros::Publisher pub_pose = node2.advertise < ratslam_ros::poses_robot
		> ("poses", 1);



image_transport::Subscriber sub1 = it.subscribe(topic_root + "/camera/image",1000, image_callback);
vector<float> posesvec;
	vector<float> landX, landY, landZ;
	double tmpX, tmpY, tmpZ;
	ifstream fileX("./landX.txt", ios::in);
	ifstream fileY("./landY.txt", ios::in);
	ifstream fileZ("./landZ.txt", ios::in);
	ifstream posesfil("./poses.txt", ios::in);
	string tmpXStr, tmpYStr, tmpZStr,tmpPosesStr;
	string tmps;
	int counter = 0;
	vector<float>  xxs;

	int rrr1;
	rrr1=rrr;
//	while (ros::ok()) {
	//	cout<<x;

		//poses.x_robot=x;
			//	poses.y_robot=y;
				//poses.theta_robot=th;


		/*while (getline(fileX, tmpXStr) && getline(fileY, tmpYStr)
				&& getline(fileZ, tmpZStr)) {
			fileX >> tmpXStr;
			landX.push_back(atof(tmpXStr.c_str()));
			fileY >> tmpYStr;
			landY.push_back(atof(tmpYStr.c_str()));
			fileZ >> tmpZStr;
			landZ.push_back(atof(tmpZStr.c_str()));
			counter++;
		}
int i=0;

		poses.x_robot=x;
		poses.y_robot=y;
		poses.theta_robot=th;
*/
//		for (int i = 0; i < 3; i++) {
//			val.valueX = rrr;
	//		val.valueY = rrr;
		//	val.valueZ = rrr;
			//landmark.landmarkData.push_back(val);
	//	}

		//pub_pose.publish(poses);
	//ros::spinOnce();
	//}
	/*landX.clear();
	landY.clear();
	landZ.clear();
	posesvec.clear();
*/
//	delete fp;
	  ros::spin();

	return 0;
}

