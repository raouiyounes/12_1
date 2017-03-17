/*
 * FPVision.cpp
 *
 *  Created on: 14 nov. 2016
 *      Author: younes
 */

#include "FPVision.h"
#include "local_view_match.h"
#include "opencv2/calib3d/calib3d_c.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
typedef vector<vector<float> > Matos;
using namespace cv;
using namespace Eigen;
using namespace std;
using namespace cv::xfeatures2d;
namespace ratslam {
FPVision::FPVision(Mat xx, int nbr_landmarks_local) {
	v_landmarks = new Matos(nbr_landmarks_local);
	nbr_of_landmarks = nbr_landmarks_local;
	index_of_landmark = 0;
	index_of_feature_pred = 0;
	testeur=0;
}
FPVision::FPVision(const FPVision &fp) {
	v_landmarks = new Matos(nbr_of_landmarks);
	v_landmarks = fp.v_landmarks;
}
FPVision::~FPVision() {
	initial_robot_pose.clear();
	delete v_landmarks;
}

/*initialize the poses of the landmarks
 * with an image tha the robot gets at the begining
 */
/*
Matos FPVision::compute_initial_pose_land(Mat initial_image) {

	float u0 = 162.0;
	float v0 = 125.0;
	float ku = 341.4;
	float kv = 261.3;
	float Suv = 255;
	float cv = 298.7;
	float cu = 236.4;
	float f = 824.4 * 0.001;
	vector < KeyPoint > surf_init;
	MatrixXd M_intrins(3, 3);
	VectorXd X_prime(3);
	vector<float> one_landmark;
	VectorXd fp_uv(3);
	M_intrins(0, 0) = ku;
	M_intrins(0, 1) = Suv;
	M_intrins(0, 2) = cu;
	M_intrins(1, 0) = 0;
	M_intrins(1, 1) = kv;
	M_intrins(1, 2) = cv;
	M_intrins(2, 0) = 0;
	M_intrins(2, 1) = 0;
	M_intrins(2, 2) = 1;
	surf_init = this->surf_extractor(initial_image);
	int i;
	KeyPoint one_feature;
	this->size_of_surf = surf_init.size();
	for (i = 0; i < surf_init.size(); i++) {
		one_feature = surf_init.at(i);
		fp_uv(0) = one_feature.pt.x;
		fp_uv(1) = one_feature.pt.y;
		X_prime = (1 / f) * M_intrins.inverse() * fp_uv;
		one_landmark.push_back(X_prime(0));

		one_landmark.push_back(X_prime(1));
		one_landmark.push_back(sqrt(pow(X_prime(0), 2) + pow(X_prime(1), 2)));
		Xv_init.push_back(one_landmark);
		one_landmark.clear();
	}



	map_of_landmarksPredicted[index_of_landmark] = Xv_init;
//index_of_landmark++;
	return Xv_init;
}
*/

Matos FPVision::compute_initial_pose_land(Mat initial_image){
float u0=162.0;
float v0=125.0;
float ku=341.4;
float kv=261.3;
float Suv=255;
float cv=298.7;
float cu=236.4;
float f=824.4*0.001;
vector<KeyPoint> surf_init;
MatrixXd M_intrins(3,3);
VectorXd X_prime(3);
vector<float> one_landmark;
Matos Xv_init;
VectorXd fp_uv(3);
M_intrins(0,0)=ku;
M_intrins(0,1)=Suv;
M_intrins(0,2)=cu;
M_intrins(1,0)=0;
M_intrins(1,1)=kv;
M_intrins(1,2)=cv;
M_intrins(2,0)=0;
M_intrins(2,1)=0;
M_intrins(2,2)=1;
surf_init=this->surf_extractor(initial_image);
int i;
KeyPoint one_feature;
this->size_of_surf=surf_init.size();
for(i=0;i<surf_init.size();i++){
	one_feature=surf_init.at(i);
fp_uv(0)=one_feature.pt.x;
fp_uv(1)=one_feature.pt.y;
X_prime=(1/f)*M_intrins.inverse()*fp_uv;
one_landmark.push_back(X_prime(0));
one_landmark.push_back(X_prime(1));
one_landmark.push_back(sqrt(pow(X_prime(0),2)+pow(X_prime(1),2)));
Xv_init.push_back(one_landmark);
one_landmark.clear();
}
map_of_landmarksPredicted[index_of_landmark]=Xv_init;
//index_of_landmark++;

return Xv_init;

}




/*perdit the pose the next sets of landmarks
 * with the use of the pihole model
 */

void FPVision::predict_feature_points() {

	float u0 = 162.0;
	float v0 = 125.0;
	float ku = 341.4;
	float kv = 261.3;
	float Suv = 255;
	float cv = 298.7;
	float cu = 236.4;
	float f = 824.4 * 0.001;

	VectorXd pose_landmarks(4);
	MatrixXd M_intrins(3, 3);
	Matos x;
	M_intrins(0, 0) = ku;
	M_intrins(0, 1) = Suv;
	M_intrins(0, 2) = cu;
	M_intrins(1, 0) = 0;
	M_intrins(1, 1) = kv;
	M_intrins(1, 2) = cv;
	M_intrins(2, 0) = 0;
	M_intrins(2, 1) = 0;
	M_intrins(2, 2) = 1;
	MatrixXd focal(3, 4);
	Matos feature_point_m_predict;
	focal(0, 0) = f;
	focal(1, 0) = 0;
	focal(2, 0) = 0;
	focal(0, 1) = 0;
	focal(1, 1) = f;
	focal(2, 1) = 0;
	focal(0, 2) = 0;
	focal(1, 2) = 0;
	focal(2, 2) = 1;
	focal(0, 3) = 0;
	focal(1, 3) = 0;
	focal(2, 3) = 0;
	x = map_of_landmarksPredicted[index_of_landmark];
	vector<float> one_fp;
	vector<float> one_landmark;
	VectorXd featurePoint(3);
	for (int i = 0; i < x.size(); i++) {
		one_landmark = x.at(i);
		pose_landmarks(0) = one_landmark.at(0);
		pose_landmarks(1) = one_landmark.at(1);
		pose_landmarks(2) = one_landmark.at(2);
		pose_landmarks(3) = 1;
		featurePoint = M_intrins * focal * pose_landmarks;
		one_fp.push_back(pose_landmarks(0));
		one_fp.push_back(pose_landmarks(1));
		one_fp.push_back(pose_landmarks(2));
		feature_point_m_predict.push_back(one_fp);
		one_fp.clear();
	}
	feature_point_predict[index_of_feature_pred] = feature_point_m_predict;
}

/*
 *predict the landmarks' pose X Y Z given the robot pose
 */
/*
void FPVision::predict_pose_landmarks(float p, float q, float delta) {


	// pose of the landmarks
	Matos visu_land_loc;
	Matos visu_land_loc_new;
	visu_land_loc =Xv_init;
	vector<float> one_landmark;
	vector<float> one_landmark_new;
	for (int i = 0; i < this->size_of_surf; i++) {
		one_landmark = visu_land_loc.at(i);


		one_landmark_new.push_back(one_landmark.at(0));
		//				- (one_landmark.at(2) - q) * sin(delta));
		one_landmark_new.push_back(one_landmark.at(1));
		one_landmark_new.push_back(
				(one_landmark.at(0) - p) * sin(delta)
						+ (one_landmark.at(2) - q) * cos(delta));
		Xv_init.push_back(one_landmark_new);
	}
	Xv_init = visu_land_loc_new;
}*/

void FPVision::predict_pose_landmarks(float p,float q,float delta){
Matos visu_land_loc;
Matos visu_land_loc_new;
visu_land_loc=map_of_landmarksPredicted[index_of_landmark];
vector<float> one_landmark;
vector<float> one_landmark_new;
for(int i=0;i<this->size_of_surf;i++){
	one_landmark=visu_land_loc.at(i);
	one_landmark_new.push_back((one_landmark.at(0)-p)*cos(delta)-(one_landmark.at(2)-q)*sin(delta));
	one_landmark_new.push_back(one_landmark.at(1));
	one_landmark_new.push_back((one_landmark.at(0)-p)*sin(delta)+(one_landmark.at(2)-q)*cos(delta));
	visu_land_loc_new.push_back(one_landmark_new);
	one_landmark_new.clear();
}

map_of_landmarksPredicted[++index_of_landmark]=visu_land_loc_new;
data=visu_land_loc_new;
visu_land_loc_new.clear();
}
/*extract the SURF features and match between two images using DMatch class
 */
std::vector<DMatch> FPVision::surf_extractor(Mat img_1, Mat imo_old) {
	//DMatch* good_matches;
	vector < vector<float> > detector_id_local;
	int minHessian = 400;
	int i;
	vector<float> line_of_features;
	Ptr < SURF > detector = SURF::create();
	detector->setHessianThreshold(minHessian);
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	Mat descriptors_1, descriptors_2;
	detector->detectAndCompute(img_1, Mat(), keypoints_1, descriptors_1);
	detector->detectAndCompute(imo_old, Mat(), keypoints_2, descriptors_2);
	FlannBasedMatcher matcher;
	std::vector < DMatch > matches;
	matcher.match(descriptors_1, descriptors_2, matches);
	double min_dist = 100.0, max_dist = 0.0;
	for (int i = 0; i < descriptors_1.rows; i++) {
		double dist = matches[i].distance;
		if (dist < min_dist)
			dist = min_dist;
		if (dist > max_dist)
			dist = max_dist;
	}
	int kl = 0;
	std::vector < DMatch > good_matches;
	for (int i = 0; i < descriptors_1.rows; i++) {
		if (matches[i].distance <= max(2 * min_dist, 0.02)) {
			good_matches.push_back(matches[i]);
		}
	}
	Mat img_matches;
	drawMatches(img_1, keypoints_1, imo_old, keypoints_2, good_matches,
			img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(),
			DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//-- Show detected matches
	imshow("Good Matches", img_matches);
	waitKey(1);
	return good_matches;
}

void FPVision::measure_depth(std::vector<KeyPoint> keypoints_1,
		std::vector<KeyPoint> keypoints_2, std::vector<DMatch> good_matches) {

	//measured depth

	vector<int> mQ1, mQ2;
	for (int i = 0; i < size_of_surf; i++) {
		mQ1.push_back(good_matches[i].queryIdx);
	}
	for (int i = 0; i < size_of_surf; i++)
		mQ2.push_back(good_matches[i].trainIdx);
	float f = 824.4 * 0.001;
	int baseline = 25;
	for (int i = 0; i < size_of_surf; i++) {
		depth.push_back(baseline * f / (mQ1.at(i) - mQ2.at(i)));

	}
}

Matos FPVision::landmarkXYandDepth(Matos data) {
	Matos landmark;
	vector<float> lineEntryLandmark;
		for(int i=0;i<data.size();i++)
	{
		lineEntryLandmark.push_back(data.at(i).at(0));
		lineEntryLandmark.push_back(data.at(i).at(1));
		lineEntryLandmark.push_back(depth.at(i));
	landmark.push_back(lineEntryLandmark);
	lineEntryLandmark.clear();
	}
	return landmark;
}

/*
 * estimate the poses of the features
 */

/*
 MatrixXd FPVision::estimateLandmarks(){

 vector<KeyPoint> keypointL=keypoint;
 MatrixXd measured_landmark(FPVision::k,3);

 for(int i=0;i<FPVision::k;i++){
 measured_landmark(i,0)=keypointL[i].pt.x;
 measured_landmark(i,1)=keypointL[i].pt.y;
 measured_landmark(i,2)=depth.at(i);
 }

 return measured_landmark;
 }*/

/*
 * exctract the SURF features
 *
 */

vector<KeyPoint> FPVision::surf_extractor(Mat img) {
	int minHessian = 400;
	int i;
	Ptr < SURF > detector = SURF::create();
	detector->setHessianThreshold(minHessian);
	std::vector < KeyPoint > keypoints_1;
	Mat descriptors_1;
	detector->detectAndCompute(img, Mat(), keypoints_1, descriptors_1);
	return keypoints_1;
}

}
