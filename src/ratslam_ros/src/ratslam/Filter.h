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

void setNLand(int);
void setNParti(int);
void setPoseCloud(MatrixXd);
void setLand(MatrixXd);

std::map<int,vector<vector<float> >> observation();
vector<vector<float> > measure(float x,float y,float theta);
vector<float> computeZObservation(vector<float> landmark,vector<float> posePartic);
std::map<int,float> dataAssociation();
float poseEstimation();
	Filter();
	virtual ~Filter();
};

} /* namespace ratslam */

#endif /* SRC_RATSLAM_ROS_SRC_RATSLAM_FILTER_H_ */
