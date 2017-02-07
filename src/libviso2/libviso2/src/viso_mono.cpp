/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/
#include <iostream>
#include <fstream>
//#include <boost/assign/std/vector.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "viso_mono.h"

//using namespace boost::assign;
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

VisualOdometryMono::VisualOdometryMono (parameters param) : param(param), VisualOdometry((VisualOdometry::parameters)param) {
}

VisualOdometryMono::~VisualOdometryMono () {
}

bool VisualOdometryMono::process (Mat image, uint8_t *I,int32_t* dims,bool replace) {
  matcher->pushBack(I,dims,replace);
  matcher->matchFeatures(0);
  matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);
  p_matched = matcher->getMatches();
  imshow("INPUT",image);
  waitKey(1);
  return updateMotion();
}

/*bool VisualOdometryMono::process (Mat image, uint8_t *I,int32_t* dims,bool replace) {

    //Ptr<ORB> detector = ORB::create();//500, 1.2f, 8, 31, 0, 4, ORB::HARRIS_SCORE, 31, 20
    //int minHessian = 400;
    //Ptr<SIFT> detector = SIFT::create( minHessian );
    Ptr<SIFT> detector = SIFT::create();
    vector<KeyPoint> curr_Keypoints;
    detector->detect(image, curr_Keypoints);
    //FAST(image, curr_Keypoints, 20, true);

    Ptr<BriefDescriptorExtractor> extractor = BriefDescriptorExtractor::create();
    Mat curr_Descriptors;
    extractor->compute(image, curr_Keypoints, curr_Descriptors);

    vector<vector<DMatch> > matches1;//, matches2;
    vector<DMatch> NNMatches1;//, NNMatches2, bestSymmetricMatchesVector;
    BFMatcher matcher(NORM_HAMMING);
    matcher.knnMatch(prev_Descriptors, curr_Descriptors, matches1, 100);

    for (size_t i(0); i < matches1.size(); i++)
    {
        float dist1 = matches1[i][0].distance;
        float dist2 = matches1[i][1].distance;

        if (dist1 < 0.8 * dist2)
        {
            NNMatches1.push_back(matches1[i][0]);
        }
    }
    p_matched.clear();
    for (size_t i(0); i < NNMatches1.size(); i++)
    {
        int queryInx = NNMatches1[i].queryIdx;
        int trainInx = NNMatches1[i].trainIdx;
        p_matched.push_back(Matcher::p_match( prev_Keypoints[queryInx].pt.x, prev_Keypoints[queryInx].pt.y,i,-1,-1,-1,
                                              curr_Keypoints[trainInx].pt.x, curr_Keypoints[trainInx].pt.y,i,-1,-1,-1 ));
    }
    for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
	circle(image, Point2f(it->u1p,it->v1p), 2, Scalar(0, 0, 255), 1);
	circle(image, Point2f(it->u1c,it->v1c), 2, Scalar(255, 0, 0), 1);
   	line(image, Point2f(it->u1p,it->v1p), Point2f(it->u1c,it->v1c), Scalar(0, 255, 0), 1, 8, 0);
    }
    prev_Keypoints.clear();
    prev_Keypoints = curr_Keypoints;
    prev_Descriptors.release();
    prev_Descriptors = curr_Descriptors.clone();
    cv::imshow("image",image);
    cv::waitKey(1);

  return updateMotion();
}*/

double VisualOdometryMono::compute_scale(Matrix R, Matrix t, Matrix K, vector<Matcher::p_match> p_matched, vector<Matcher::p_match> p_p_matched, Matrix p_Tr_delta, double &ll) {

	/*if (p_p_matched.size()==0) {
	    ll = 1.0;
	    return 1.0;
	}

	Matrix X1(4,1),X2(4,1);
	Matrix P1 = p_Tr_delta.getMat(0,0,2,3);
	Matrix P2(3,4);
	P2.setMat(R,0,0);
	P2.setMat(t,0,3);
	//vector<Matcher::p_match> matched1, matched2;
	double num = 0.0;
	double den = 0.0;
	int32_t index = 0;
	//for (vector<Matcher::p_match>::iterator it2 = p_matched.begin(); it2!=p_matched.end(); it2++) {
	    //for (vector<Matcher::p_match>::iterator it1 = p_p_matched.begin(); it1!=p_p_matched.end(); it1++) {
	for (int32_t i=0;i<p_matched.size();i++) {
	    for (int32_t j=0;j<p_p_matched.size();j++) {
		if(p_matched.at(i).u1p == p_p_matched.at(j).u1c && p_matched.at(i).v1p == p_p_matched.at(j).v1c ) {
		    //matched_1.push_back(Matcher::p_match(it2->u1p,it2->v1p,it2->u1c,it2->v1c));
		    //matched_2.push_back(Matcher::p_match(it1->u1p,it1->v1p,it1->u1c,it1->v1c));
		    if(triangulate(p_p_matched.at(j),K,P1,X1) && triangulate(p_matched.at(i),K,P2,X2) && X1.val[3][0]!=0 && X2.val[3][0]!=0){
			Matrix  X2_ = p_Tr_delta*X1;
			double len1 = sqrt((X2_.val[0][0]/X2_.val[3][0])*(X2_.val[0][0]/X2_.val[3][0]) +
					   (X2_.val[1][0]/X2_.val[3][0])*(X2_.val[1][0]/X2_.val[3][0]) +
					   (X2_.val[2][0]/X2_.val[3][0])*(X2_.val[2][0]/X2_.val[3][0]) );
			double len2 = sqrt((X2.val[0][0]/X2.val[3][0])*(X2.val[0][0]/X2.val[3][0]) +
					   (X2.val[1][0]/X2.val[3][0])*(X2.val[1][0]/X2.val[3][0]) +
					   (X2.val[2][0]/X2.val[3][0])*(X2.val[2][0]/X2.val[3][0]) );
			double w = weight(p_p_matched.at(j),p_matched.at(i),K,P1,P2);
			num = num + w*(len1/len2);
			den = den + len1/len2;
			index++;
		    }
		}
	    }
	}
	if(index==0) { ll=1.0; return ll; }
	else { ll = den/index; return num/den; }*/
return 1.0;
}//compute_scale

int32_t VisualOdometryMono::triangulate (Matrix K, Matrix R, Matrix t, vector<Matcher::p_match> p_matched, vector< vector<Point2d> > &pointsImg, vector<Point3d> &points3D) {

  // init 3d point matrix
  Matrix X;
  X = Matrix(4,p_matched.size());
  
  // projection matrices
  Matrix P1(3,4);
  Matrix P2(3,4);
  P1.setMat(K,0,0);
  P2.setMat(R,0,0);
  P2.setMat(t,0,3);
  P2 = K*P2;
  
  // triangulation via orthogonal regression
  Matrix J(4,4);
  Matrix U,S,V;
  int index=0;
  vector<int> pos_idx;
  for (int32_t i=0; i<(int)p_matched.size(); i++) {
    for (int32_t j=0; j<4; j++) {
      J.val[0][j] = P1.val[2][j]*p_matched[i].u1p - P1.val[0][j];
      J.val[1][j] = P1.val[2][j]*p_matched[i].v1p - P1.val[1][j];
      J.val[2][j] = P2.val[2][j]*p_matched[i].u1c - P2.val[0][j];
      J.val[3][j] = P2.val[2][j]*p_matched[i].v1c - P2.val[1][j];
    }
    J.svd(U,S,V);
    if (S.val[2][0] - S.val[3][0] > 0.01) {//>=
      X.setMat(V.getMat(0,3,3,3),0,index);
      pos_idx.push_back(i);
      index++;
    }
  }
  
  // compute inliers
  Matrix  AX1 = P1*X;
  Matrix  BX1 = P2*X;
  Matrix Xnorm = X/X.getMat(3,0,3,-1);
  //pointsImg.resize(2);
  int32_t num = 0;
  //int32_t indexCorrection = 0;
  for (int32_t i=0; i<X.n; i++) {
    if (AX1.val[2][i]*X.val[3][i]>0 && BX1.val[2][i]*X.val[3][i]>0 && Xnorm.val[2][i]>0) {
      points3D.push_back ( Point3d(X.val[0][i], X.val[1][i], X.val[2][i]) );
      pointsImg[0].push_back ( Point2d(p_matched[pos_idx[i]].u1p, p_matched[pos_idx[i]].v1p) );
      pointsImg[1].push_back ( Point2d(p_matched[pos_idx[i]].u1c, p_matched[pos_idx[i]].v1c) );
      num++;
    }
    /*else {
      prev_points.erase (prev_points.begin() + (i - indexCorrection));
      curr_points.erase (curr_points.begin() + (i - indexCorrection));
      indexCorrection++;
    }*/
  }

  // return number of inliers
  return num;
}

/*int32_t 3_view( vector<Matcher::p_match> p_matched, vector<Matcher::p_match> p_p_matched, vector<Point3d> &points3D, vector< vector<Point2d> > *pointsImg) {

    if (p_p_matched.size()==0) return 0;

    Matrix X1(4,1), X2(4,1);
    Matrix P1 = p_Tr_delta.getMat(0,0,2,3);
    Matrix P2(3,4);
    P2.setMat(R,0,0);
    P2.setMat(t,0,3);
    for (int32_t i=0; i<p_matched.size(); i++) {
	for (int32_t j=0; j<p_p_matched.size(); j++) {
	    if(p_matched.at(i).u1p == p_p_matched.at(j).u1c && p_matched.at(i).v1p == p_p_matched.at(j).v1c ) {
		//matched_1.push_back(Matcher::p_match(it2->u1p,it2->v1p,it2->u1c,it2->v1c));
		//matched_2.push_back(Matcher::p_match(it1->u1p,it1->v1p,it1->u1c,it1->v1c));
		if(triangulate(p_p_matched.at(j),K,P1,X1) && triangulate(p_matched.at(i),K,P2,X2) && X1.val[3][0]!=0 && X2.val[3][0]!=0) {
		    Matrix  P3 = Tr_delta*p_Tr_delta;
		    points3D.push_back(Point3d(X1.val[0][0], X1.val[1][0], X1.val[2][0]));
		    pointsImg.
		}
	    }
	}
    }
}*/

double VisualOdometryMono::weight(Matcher::p_match it1, Matcher::p_match it2, Matrix K, Matrix P1, Matrix P2) {

	double data_x1[3] = {it1.u1p, it1.v1p, 1.0};
	double data_x2[3] = {it1.u1c, it1.v1c, 1.0};
	double data_x3[3] = {it2.u1c, it2.v1c, 1.0};
	Matrix x1(3,1,data_x1);
	Matrix x2(3,1,data_x2);
	Matrix x3(3,1,data_x3);

	Matrix R(3,3);
	Matrix rayrot(3,1);
	double nx,ny,rayangle1,rayangle2;

	R = P1.getMat(0,0,2,2);
	rayrot = R*K.inv(K)*x1;
	nx = rayrot.l2norm();
	ny = x2.l2norm();
	rayangle1 = acos((rayrot.val[0][0]*x2.val[0][0] + rayrot.val[1][0]*x2.val[1][0] + rayrot.val[2][0]*x2.val[2][0]) / (nx*ny));

	R = P2.getMat(0,0,2,2);
	rayrot = R*K.inv(K)*x2;
	nx = rayrot.l2norm();
	ny = x3.l2norm();
	rayangle2 = acos((rayrot.val[0][0]*x3.val[0][0] + rayrot.val[1][0]*x3.val[1][0] + rayrot.val[2][0]*x3.val[2][0]) / (nx*ny));

	return (tan(rayangle1/2.0) * tan(rayangle2/2.0));	

}//weight

Matrix VisualOdometryMono::estimateMotion2 (vector<Matcher::p_match> p_matched) {}

//VisualOdometryMono::estimateMotion (vector<Matcher::p_match> p_matched) {}

vector<double> VisualOdometryMono::estimateMotion (vector<Matcher::p_match> p_matched) {

  int32_t N = p_matched.size();
  if (N<10)
    return vector<double>();

  vector<Point2f> prev_points, curr_points;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    prev_points.push_back(Point2f(it->u1p,it->v1p));
    curr_points.push_back(Point2f(it->u1c,it->v1c));
  }
  Mat F_(3,3,CV_64FC1), mask, R_(3,3,CV_64FC1), t_(3,1,CV_64FC1); 
  //E_ = findEssentialMat(prev_points, curr_points, param.calib.f, Point2d(param.calib.cu,param.calib.cv), RANSAC, 0.999, 1.0, mask);
  //recoverPose(E_, prev_points, curr_points, R_, t_, param.calib.f, Point2d(param.calib.cu,param.calib.cv), mask);
  F_ = findFundamentalMat(prev_points, curr_points, FM_RANSAC, 1, 0.999, mask);

  double K_data[9] = {param.calib.f,0,param.calib.cu,0,param.calib.f,param.calib.cv,0,0,1};
  Matrix K(3,3,K_data);

  /*double E_data[9] = {E_.at<double>(0,0), E_.at<double>(0,1), E_.at<double>(0,2),
			E_.at<double>(1,0), E_.at<double>(1,1), E_.at<double>(1,2),
			E_.at<double>(2,0), E_.at<double>(2,1), E_.at<double>(2,2) };
  Matrix E(3,3,E_data);*/

  double F_data[9] = {	F_.at<double>(0,0), F_.at<double>(0,1), F_.at<double>(0,2),
			F_.at<double>(1,0), F_.at<double>(1,1), F_.at<double>(1,2),
			F_.at<double>(2,0), F_.at<double>(2,1), F_.at<double>(2,2) };
  Matrix F(3,3,F_data);

  //double t_data[3] = { t_.at<double>(0,0), t_.at<double>(1,0), t_.at<double>(2,0) };
  //Matrix t(3,1,t_data);

  Matrix E;
  E = ~K*F*K;
  Matrix U,W,V;
  E.svd(U,W,V);
  W.val[2][0] = 0;
  E = U*Matrix::diag(W)*~V;
  
  // compute 3d points X and R|t up to scale
  Matrix X_,X,R,t;
  EtoRt(E,K,p_matched,X_,R,t);
  /*recoverPose(E_, prev_points, curr_points, R_, t_, param.calib.f, Point2d(param.calib.cu,param.calib.cv), mask);

  double R_data[9] = {	R_.at<double>(0,0), R_.at<double>(0,1), R_.at<double>(0,2),
			R_.at<double>(1,0), R_.at<double>(1,1), R_.at<double>(1,2),
			R_.at<double>(2,0), R_.at<double>(2,1), R_.at<double>(2,2) };
  Matrix R(3,3,E_data);
  double t_data[3] = { t_.at<double>(0,0), t_.at<double>(1,0), t_.at<double>(2,0) };
  Matrix t(3,1,t_data);*/

  // normalize 3d points and remove points behind image plane
  X = X_/X_.getMat(3,0,3,-1);
  vector<int32_t> pos_idx;
  for (int32_t i=0; i<X.n; i++)
    if (X.val[2][i]>0)
      pos_idx.push_back(i);	
  Matrix X_plane = X.extractCols(pos_idx);

  // we need at least 10 points to proceed
  if (X_plane.n<10)
    return vector<double>();
  
  // get elements closer than median
  double median;
  smallerThanMedian(X_plane,median);
  
  // return error on large median (litte motion)
  if (median>param.motion_threshold)
    return vector<double>();

  // project features to 2d
  Matrix x_plane(2,X_plane.n);
  x_plane.setMat(X_plane.getMat(1,0,2,-1),0,0);
  
  Matrix n(2,1);
  n.val[0][0]       = cos(-param.pitch);
  n.val[1][0]       = sin(-param.pitch);
  Matrix   d        = ~n*x_plane;
  double   sigma    = median/50.0;
  double   weight   = 1.0/(2.0*sigma*sigma);
  double   best_sum = 0;
  int32_t  best_idx = 0;

  // find best plane
  for (int32_t i=0; i<x_plane.n; i++) {
    if (d.val[0][i]>median/param.motion_threshold) {
      double sum = 0;
      for (int32_t j=0; j<x_plane.n; j++) {
        double dist = d.val[0][j]-d.val[0][i];
        sum += exp(-dist*dist*weight);
      }
      if (sum>best_sum) {
        best_sum = sum;
        best_idx = i;
      }
    }
  }
  t = t*param.height/d.val[0][best_idx];

  // compute rotation angles
  double ry = asin(R.val[0][2]);
  double rx = asin(-R.val[1][2]/cos(ry));
  double rz = asin(-R.val[0][1]/cos(ry));

  // return parameter vector
  vector<double> tr_delta;
  tr_delta.resize(6);
  tr_delta[0] = rx;
  tr_delta[1] = ry;
  tr_delta[2] = rz;
  tr_delta[3] = t.val[0][0];
  tr_delta[4] = t.val[1][0];
  tr_delta[5] = t.val[2][0];
  return tr_delta;
}

/*vector<double> VisualOdometryMono::estimateMotion (vector<Matcher::p_match> p_matched) {

  // get number of matches
  int32_t N = p_matched.size();
  if (N<10)
    return vector<double>();
   
  // create calibration matrix
  double K_data[9] = {param.calib.f,0,param.calib.cu,0,param.calib.f,param.calib.cv,0,0,1};
  Matrix K(3,3,K_data);
    
  // normalize feature points and return on errors
  Matrix Tp,Tc;
  vector<Matcher::p_match> p_matched_normalized = p_matched;
  if (!normalizeFeaturePoints(p_matched_normalized,Tp,Tc))
    return vector<double>();

  // initial RANSAC estimate of F
  Matrix E,F;
  inliers.clear();
  for (int32_t k=0;k<param.ransac_iters;k++) {

    // draw random sample set
    vector<int32_t> active = getRandomSample(N,8);

    // estimate fundamental matrix and get inliers
    fundamentalMatrix(p_matched_normalized,active,F);
    vector<int32_t> inliers_curr = getInlier(p_matched_normalized,F);

    // update model if we are better
    if (inliers_curr.size()>inliers.size())
      inliers = inliers_curr;
  }
  
  // are there enough inliers?
  if (inliers.size()<10)
    return vector<double>();
  
  // refine F using all inliers
  fundamentalMatrix(p_matched_normalized,inliers,F); 
  
  // denormalise and extract essential matrix
  F = ~Tc*F*Tp;
  E = ~K*F*K;
  
  // re-enforce rank 2 constraint on essential matrix
  Matrix U,W,V;
  E.svd(U,W,V);
  W.val[2][0] = 0;
  E = U*Matrix::diag(W)*~V;
  
  // compute 3d points X and R|t up to scale
  Matrix X,R,t;
  EtoRt(E,K,p_matched,X,R,t);
  
  // normalize 3d points and remove points behind image plane
  X = X/X.getMat(3,0,3,-1);
  vector<int32_t> pos_idx;
  for (int32_t i=0; i<X.n; i++)
    if (X.val[2][i]>0)
      pos_idx.push_back(i);
  Matrix X_plane = X.extractCols(pos_idx);
  
  // we need at least 10 points to proceed
  if (X_plane.n<10)
    return vector<double>();
  
  // get elements closer than median
  double median;
  smallerThanMedian(X_plane,median);
  
  // return error on large median (litte motion)
  if (median>param.motion_threshold)
    return vector<double>();
  
  // project features to 2d
  Matrix x_plane(2,X_plane.n);
  x_plane.setMat(X_plane.getMat(1,0,2,-1),0,0);
  
  Matrix n(2,1);
  n.val[0][0]       = cos(-param.pitch);
  n.val[1][0]       = sin(-param.pitch);
  Matrix   d        = ~n*x_plane;
  double   sigma    = median/50.0;
  double   weight   = 1.0/(2.0*sigma*sigma);
  double   best_sum = 0;
  int32_t  best_idx = 0;

  // find best plane
  for (int32_t i=0; i<x_plane.n; i++) {
    if (d.val[0][i]>median/param.motion_threshold) {
      double sum = 0;
      for (int32_t j=0; j<x_plane.n; j++) {
        double dist = d.val[0][j]-d.val[0][i];
        sum += exp(-dist*dist*weight);
      }
      if (sum>best_sum) {
        best_sum = sum;
        best_idx = i;
      }
    }
  }
  t = t*param.height/d.val[0][best_idx];
  
  // compute rotation angles
  double ry = asin(R.val[0][2]);
  double rx = asin(-R.val[1][2]/cos(ry));
  double rz = asin(-R.val[0][1]/cos(ry));
  
  // return parameter vector
  vector<double> tr_delta;
  tr_delta.resize(6);
  tr_delta[0] = rx;
  tr_delta[1] = ry;
  tr_delta[2] = rz;
  tr_delta[3] = t.val[0][0];
  tr_delta[4] = t.val[1][0];
  tr_delta[5] = t.val[2][0];
  return tr_delta;
}*/

Matrix VisualOdometryMono::smallerThanMedian (Matrix &X,double &median) {
  
  // set distance and index vector
  vector<double> dist;
  vector<int32_t> idx;
  for (int32_t i=0; i<X.n; i++) {
    dist.push_back(fabs(X.val[0][i])+fabs(X.val[1][i])+fabs(X.val[2][i]));
    idx.push_back(i);
  }
  
  // sort elements
  sort(idx.begin(),idx.end(),idx_cmp<vector<double>&>(dist));
  
  // get median
  int32_t num_elem_half = idx.size()/2;
  median = dist[idx[num_elem_half]];
  
  // create matrix containing elements closer than median
  Matrix X_small(4,num_elem_half+1);
  for (int32_t j=0; j<=num_elem_half; j++)
    for (int32_t i=0; i<4; i++)
      X_small.val[i][j] = X.val[i][idx[j]];
	return X_small;
}

bool VisualOdometryMono::normalizeFeaturePoints(vector<Matcher::p_match> &p_matched,Matrix &Tp,Matrix &Tc) {
  
  // shift origins to centroids
  double cpu=0,cpv=0,ccu=0,ccv=0;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    cpu += it->u1p;
    cpv += it->v1p;
    ccu += it->u1c;
    ccv += it->v1c;
  }
  cpu /= (double)p_matched.size();
  cpv /= (double)p_matched.size();
  ccu /= (double)p_matched.size();
  ccv /= (double)p_matched.size();
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p -= cpu;
    it->v1p -= cpv;
    it->u1c -= ccu;
    it->v1c -= ccv;
  }
  
  // scale features such that mean distance from origin is sqrt(2)
  double sp=0,sc=0;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    sp += sqrt(it->u1p*it->u1p+it->v1p*it->v1p);
    sc += sqrt(it->u1c*it->u1c+it->v1c*it->v1c);
  }
  if (fabs(sp)<1e-10 || fabs(sc)<1e-10)
    return false;
  sp = sqrt(2.0)*(double)p_matched.size()/sp;
  sc = sqrt(2.0)*(double)p_matched.size()/sc;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p *= sp;
    it->v1p *= sp;
    it->u1c *= sc;
    it->v1c *= sc;
  }
  
  // compute corresponding transformation matrices
  double Tp_data[9] = {sp,0,-sp*cpu,0,sp,-sp*cpv,0,0,1};
  double Tc_data[9] = {sc,0,-sc*ccu,0,sc,-sc*ccv,0,0,1};
  Tp = Matrix(3,3,Tp_data);
  Tc = Matrix(3,3,Tc_data);
  
  // return true on success
  return true;
}

void VisualOdometryMono::fundamentalMatrix (const vector<Matcher::p_match> &p_matched,const vector<int32_t> &active,Matrix &F) {
  
  // number of active p_matched
  int32_t N = active.size();
  
  // create constraint matrix A
  Matrix A(N,9);
  for (int32_t i=0; i<N; i++) {
    Matcher::p_match m = p_matched[active[i]];
    A.val[i][0] = m.u1c*m.u1p;
    A.val[i][1] = m.u1c*m.v1p;
    A.val[i][2] = m.u1c;
    A.val[i][3] = m.v1c*m.u1p;
    A.val[i][4] = m.v1c*m.v1p;
    A.val[i][5] = m.v1c;
    A.val[i][6] = m.u1p;
    A.val[i][7] = m.v1p;
    A.val[i][8] = 1;
  }
   
  // compute singular value decomposition of A
  Matrix U,W,V;
  A.svd(U,W,V);
   
  // extract fundamental matrix from the column of V corresponding to the smallest singular value
  F = Matrix::reshape(V.getMat(0,8,8,8),3,3);
  
  // enforce rank 2
  F.svd(U,W,V);
  W.val[2][0] = 0;
  F = U*Matrix::diag(W)*~V;
}

vector<int32_t> VisualOdometryMono::getInlier (vector<Matcher::p_match> &p_matched,Matrix &F) {

  // extract fundamental matrix
  double f00 = F.val[0][0]; double f01 = F.val[0][1]; double f02 = F.val[0][2];
  double f10 = F.val[1][0]; double f11 = F.val[1][1]; double f12 = F.val[1][2];
  double f20 = F.val[2][0]; double f21 = F.val[2][1]; double f22 = F.val[2][2];
  
  // loop variables
  double u1,v1,u2,v2;
  double x2tFx1;
  double Fx1u,Fx1v,Fx1w;
  double Ftx2u,Ftx2v;
  
  // vector with inliers
  vector<int32_t> inliers;
  
  // for all matches do
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++) {

    // extract matches
    u1 = p_matched[i].u1p;
    v1 = p_matched[i].v1p;
    u2 = p_matched[i].u1c;
    v2 = p_matched[i].v1c;
    
    // F*x1
    Fx1u = f00*u1+f01*v1+f02;
    Fx1v = f10*u1+f11*v1+f12;
    Fx1w = f20*u1+f21*v1+f22;
    
    // F'*x2
    Ftx2u = f00*u2+f10*v2+f20;
    Ftx2v = f01*u2+f11*v2+f21;
    
    // x2'*F*x1
    x2tFx1 = u2*Fx1u+v2*Fx1v+Fx1w;
    
    // sampson distance
    double d = x2tFx1*x2tFx1 / (Fx1u*Fx1u+Fx1v*Fx1v+Ftx2u*Ftx2u+Ftx2v*Ftx2v);
    
    // check threshold
    if (fabs(d)<param.inlier_threshold)
      inliers.push_back(i);
  }

  // return set of all inliers
  return inliers;
}

void VisualOdometryMono::EtoRt(Matrix &E,Matrix &K,vector<Matcher::p_match> &p_matched,Matrix &X,Matrix &R,Matrix &t) {

  // hartley matrices
  double W_data[9] = {0,-1,0,+1,0,0,0,0,1};
  double Z_data[9] = {0,+1,0,-1,0,0,0,0,0};
  Matrix W(3,3,W_data);
  Matrix Z(3,3,Z_data); 
  
  // extract T,R1,R2 (8 solutions)
  Matrix U,S,V;
  E.svd(U,S,V);
  Matrix T  = U*Z*~U;
  Matrix Ra = U*W*(~V);
  Matrix Rb = U*(~W)*(~V);
  
  // convert T to t
  t = Matrix(3,1);
  t.val[0][0] = T.val[2][1];
  t.val[1][0] = T.val[0][2];
  t.val[2][0] = T.val[1][0];
  
  // assure determinant to be positive
  if (Ra.det()<0) Ra = -Ra;
  if (Rb.det()<0) Rb = -Rb;
  
  // create vector containing all 4 solutions
  vector<Matrix> R_vec;
  vector<Matrix> t_vec;
  R_vec.push_back(Ra); t_vec.push_back( t);
  R_vec.push_back(Ra); t_vec.push_back(-t);
  R_vec.push_back(Rb); t_vec.push_back( t);
  R_vec.push_back(Rb); t_vec.push_back(-t);
  
  // try all 4 solutions
  Matrix X_curr;
  int32_t max_inliers = 0;
  for (int32_t i=0; i<4; i++) {
    int32_t num_inliers = triangulateChieral(p_matched,K,R_vec[i],t_vec[i],X_curr);
    if (num_inliers>max_inliers) {
      max_inliers = num_inliers;
      X = X_curr;
      R = R_vec[i];
      t = t_vec[i];
    }
  }
}

int32_t VisualOdometryMono::triangulateChieral (vector<Matcher::p_match> &p_matched,Matrix &K,Matrix &R,Matrix &t,Matrix &X) {
  
  // init 3d point matrix
  X = Matrix(4,p_matched.size());
  
  // projection matrices
  Matrix P1(3,4);
  Matrix P2(3,4);
  P1.setMat(K,0,0);
  P2.setMat(R,0,0);
  P2.setMat(t,0,3);
  P2 = K*P2;
  
  // triangulation via orthogonal regression
  Matrix J(4,4);
  Matrix U,S,V;
  for (int32_t i=0; i<(int)p_matched.size(); i++) {
    for (int32_t j=0; j<4; j++) {
      J.val[0][j] = P1.val[2][j]*p_matched[i].u1p - P1.val[0][j];
      J.val[1][j] = P1.val[2][j]*p_matched[i].v1p - P1.val[1][j];
      J.val[2][j] = P2.val[2][j]*p_matched[i].u1c - P2.val[0][j];
      J.val[3][j] = P2.val[2][j]*p_matched[i].v1c - P2.val[1][j];
    }
    J.svd(U,S,V);
    X.setMat(V.getMat(0,3,3,3),0,i);
  }
  
  // compute inliers
  Matrix  AX1 = P1*X;
  Matrix  BX1 = P2*X;
  int32_t num = 0;
  for (int32_t i=0; i<X.n; i++)
    if (AX1.val[2][i]*X.val[3][i]>0 && BX1.val[2][i]*X.val[3][i]>0)
      num++;
  
  // return number of inliers
  return num;
}
