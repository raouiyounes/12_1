/*
 * Filter.cpp
 *
 *  Created on: 14 mars 2017
 *      Author: younes
 */

#include "Filter.h"
#include <eigen3/Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace std;
namespace ratslam {

Filter::Filter() {
	// TODO Auto-generated constructor stub

}

Filter::~Filter() {
	// TODO Auto-generated destructor stub
}



/*
 * compute the observation of each particle and each landmark
 */
map<int,vector<vector<float> >> Filter::observation(){
int i,j;
map<int, vector<vector<float> >> mapLandmark;
vector<float> XPartic;
vector<float> landmark;
vector<float> Z;
vector<vector<float> >ZParticle;
MatrixXd observationM(this->numbeOfLandmarks,this->numberOfPartic);
	for(i=0;i<this->numbeOfLandmarks;i++){
		for(i=0;i<this->numberOfPartic;i++){
		XPartic.push_back(this->robotPosePart(j,0));
XPartic.push_back(this->robotPosePart(j,1));
XPartic.push_back(this->robotPosePart(j,2));
landmark.push_back(this->landmarks(i,0));
landmark.push_back(this->landmarks(i,1));
Z=this->computeZObservation(landmark,XPartic);
ZParticle.push_back(Z);
	}
		mapLandmark[i]=ZParticle;

}
	return mapLandmark;
}
vector<float> Filter::computeZObservation(vector<float> landmark,vector<float> posePartic){
vector<float> Z;
Z.push_back(sqrt(pow(landmark.at(0)-posePartic.at(0),2)+pow(landmark.at(1)-posePartic.at(1),1)));
Z.push_back(atan((landmark.at(1)-posePartic.at(1))/(landmark.at(0)-posePartic.at(0)))-posePartic.at(2))
return Z;
}
/*
 * measure from the true pose to all landmarks
 */
vector<vector<float> > Filter::measure(float x,float y,float theta){
int i;
	vector<float> XTrue;
	vector<float> landmark;
	XTrue={x,y,theta};
vector<vector<float> > measure;
vector<float> ZMeasure;
	for(i=0;i<this->numbeOfLandmarks;i++){
landmark.push_back(this->landmarks(i,0));
landmark.push_back(this->landmarks(i,1));
		ZMeasure=this->computeZObservation(landmark,XTrue);
		measure.push_back(ZMeasure);
	}
return measure;
}


maps<int,float> dataAssociation();




float poseEstimation();




} /* namespace ratslam */
