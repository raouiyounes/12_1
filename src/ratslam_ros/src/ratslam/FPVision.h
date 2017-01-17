/*
 * FPVision.h
 *
 *  Created on: 14 nov. 2016
 *      Author: younes
 */

#ifndef SRC_RATSLAM_ROS_SRC_RATSLAM_FPVISION_H_
#include "local_view_match.h"
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include <eigen3/Eigen/Dense>
#include <map>
#define SRC_RATSLAM_ROS_SRC_RATSLAM_FPVISION_H_
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
using namespace Eigen;

typedef vector<vector<float> > Matos;
namespace ratslam{
class FPVision {
int index_of_landmark;
int index_of_feature_pred;
vector<vector<float> > *v_landmarks;
vector<vector <float> >  detector_id;
int nbr_of_landmarks;
// get from the odometry
vector<float> initial_robot_pose;

public:
map<int,Matos>  map_of_landmarks;
map<int,Matos> feature_point_predict;
MatrixXd init_landmark_pose;
public:
static int k;
int size_of_surf;
Matos Visu_landmark_coordina;


FPVision(Mat,int);
FPVision(const FPVision &fp);
vector<KeyPoint> surf_extractor(Mat img);
DMatch* surf_extractor(Mat,Mat);
void matching(vector<vector<double> > &);
void compute_landmarks(std::vector<KeyPoint>  keypoints);
virtual ~FPVision();
void compute_depth(std::vector<KeyPoint> , std::vector<KeyPoint> ,DMatch* );
Matos compute_initial_pose_land(Mat);
void predict_pose_landmarks(float p,float q,float delta);
void predict_feature_points();

};

}
#endif /* SRC_RATSLAM_ROS_SRC_RATSLAM_FPVISION_H_ */
