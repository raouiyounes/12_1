#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <ratslam_ros/toute.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
//#include "utils/visuPCL.cpp"

#include <iostream>
using namespace pcl;
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");



class cloudHandler
{
public:
    cloudHandler():viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("/camera/depth_registered/points", 10, &cloudHandler::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB,this);
    }
    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg (input, cloud);
        viewer.showCloud(cloud.makeShared());
    }
    void timerCB(const ros::TimerEvent&)
    {
        if(viewer.wasStopped())
            {
                ros::shutdown();
            }
    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
};

void chatterCallback(ratslam_ros::toute::ConstPtr  msg)
{

	  ROS_INFO("Idddd heard: [%f]", msg->landX[0]);

	/*
int ttt;
	for(int i=0;i<msg->landmarkData.size();i++){
	      const ratslam_ros::scalarMess data = msg->landmarkData[i];
  ROS_INFO("I heard: [%f]", msg->landmarkData.at(i).valueY);
cin>>ttt;
  cout<<endl;
	}

	pcl::PointCloud<pcl::PointXYZ> cloud;
	  // Fill in the cloud data
	  cloud.width    = msg->landmarkData.size();
	  cloud.height   = 3;
	  cloud.is_dense = false;
	  cloud.points.resize (cloud.width * cloud.height);

	  for (size_t i = 0; i < msg->landmarkData.size(); ++i)
	  {
	    cloud.points[i].x = msg->landmarkData.at(i).valueX;
	    cloud.points[i].y = msg->landmarkData.at(i).valueY;
	    cloud.points[i].z = msg->landmarkData.at(i).valueZ;
	  }
	  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
		    pcl::io::loadPCDFile ("my_point_cloud.pcd", *cloud1);
		    //test("my_point_cloud.pcd");

		    //blocks until the cloud is actually rendered
		    viewer.showCloud(cloud1);
		  while (!viewer.wasStopped())
		  	    {
		  	    }

	*/
	  }
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_visualize");
   // cloudHandler handler;
    ros::NodeHandle n;
    ros::Subscriber sub_template = n.subscribe<ratslam_ros::toute>("landmark", 1000,chatterCallback);
    ros::spin();
    return 0;
}
