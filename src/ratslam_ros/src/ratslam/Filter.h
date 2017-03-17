/*
 * Filter.h
 *
 *  Created on: 14 mars 2017
 *      Author: younes
 */
#ifndef SRC_RATSLAM_ROS_SRC_RATSLAM_FILTER_H_
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
#define SRC_RATSLAM_ROS_SRC_RATSLAM_FILTER_H_
using namespace Eigen;
using namespace std;

namespace ratslam {

class Filter {
int numbeOfLandmarks;
int numberOfPartic;
MatrixXd robotPosePart;
MatrixXd landmarks;
public:
std::map<int,vector<vector<float> >> observation();
MatrixXd measure();
vector<double> computeZObservation(vector<float> landmark,vector<float> posePartic);
std::map<int,float> dataAssociation();
float poseEstimation();
	Filter();
	virtual ~Filter();
};

} /* namespace ratslam */

#endif /* SRC_RATSLAM_ROS_SRC_RATSLAM_FILTER_H_ */
