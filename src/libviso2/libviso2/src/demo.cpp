/*
Copyright 2012. All rights reserved.
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

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdint.h>

//#include <viso_stereo.h>
//#include <png++/png.hpp>

#include <viso_mono.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

/*Ptr<ORB> detector = ORB::create(/*param.nombre_features, 1.2f, 8, 31, 0, 4, ORB::HARRIS_SCORE, 31, 20*);
Ptr<BriefDescriptorExtractor> extractor = BriefDescriptorExtractor::create();
BFMatcher matcher(NORM_HAMMING);
Mat prev_descriptor;
vector<KeyPoint> prev_keypoint;*/
Mat traj = Mat::ones(800, 1200, CV_8UC3);

void show_trajectory(Matrix C) {

	char text[100];
  	Point textOrg(10, 50);
	int x = int(C.val[2][3]) +300;
    	int y = int(C.val[0][3]) +300;
    	circle(traj, Point(x, y) ,1, CV_RGB(255,255,255), 1);
    	imshow( "Trajectory", traj);
    	waitKey(1);
}

/*void feature_detection(Mat image, vector<KeyPoint> &key_point, Mat &desc) {

    vector<KeyPoint> key_point;
    Mat desc;
    detector->detect(image, key_point);
    extractor->compute(image, key_point, desc);
}*/

/*void feature_matching(/*vector<KeyPoint> key_point, Mat desc/Mat image, vector<Matcher::p_match> &p_matched_) {

    vector<KeyPoint> key_point;
    Mat desc;
    detector->detect(image, key_point);
    extractor->compute(image, key_point, desc);

    vector<vector<DMatch> > matches1, matches2;
    vector<DMatch> NNMatches1, NNMatches2, bestSymmetricMatchesVector;
    matcher.knnMatch(prev_descriptor, desc, matches1, 100);
    matcher.knnMatch(desc, prev_descriptor, matches2, 100);

    for (size_t i(0); i < matches1.size(); i++)
    {
        float dist1 = matches1[i][0].distance;
        float dist2 = matches1[i][1].distance;

        if (dist1 < 0.8 * dist2)
        {
            NNMatches1.push_back(matches1[i][0]);
        }
    }
    for (size_t i(0); i < matches2.size(); i++)
    {
        float dist1 = matches2[i][0].distance;
        float dist2 = matches2[i][1].distance;

        if (dist1 < 0.8 * dist2)
        {
            NNMatches2.push_back(matches2[i][0]);
        }
    }

    for (size_t i(0); i < NNMatches1.size(); i++)
    {
        for (size_t j(0); j < NNMatches2.size(); j++)
        {
            if (NNMatches1[i].queryIdx == NNMatches2[j].trainIdx && NNMatches1[i].trainIdx == NNMatches2[j].queryIdx)
            {
                bestSymmetricMatchesVector.push_back(
                    DMatch(NNMatches1[i].queryIdx, NNMatches1[i].trainIdx, NNMatches1[i].distance));
                break;
            }
        }
    }
    for (size_t i(0); i < bestSymmetricMatchesVector/*NNMatches1/.size(); i++)
    {
        int queryInx = bestSymmetricMatchesVector/*NNMatches1/[i].queryIdx;
        int trainInx = bestSymmetricMatchesVector/*NNMatches1/[i].trainIdx;
        //prev_points.push_back(Point2f(prev_Keypoints[queryInx].pt.x, prev_Keypoints[queryInx].pt.y));
        //curr_points.push_back(Point2f(curr_Keypoints[trainInx].pt.x, curr_Keypoints[trainInx].pt.y));
	Matcher::p_match m;
	m.u1p = float(prev_keypoint[queryInx].pt.x);
	m.v1p = float(prev_keypoint[queryInx].pt.y);
	m.u1c = float(key_point[trainInx].pt.x);
	m.v1c = float(key_point[trainInx].pt.y);
	p_matched_.push_back(m);
    }
    prev_descriptor.release(); prev_descriptor = desc;
    prev_keypoint.clear(); prev_keypoint = key_point;
}*/

int main (int argc, char** argv) {

  // we need the path name to 2010_03_09_drive_0019 as input argument
  if (argc<2) {
    cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
    return 1;
  }

  // sequence directory
  string dir = argv[1];
  
  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_stereo.h
  VisualOdometryMono::parameters param;
  
  // calibration parameters for sequence 2010_03_09_drive_0019 
  // f[pixels] = image_width[pixels]*f[mm]/CCD_width[mm]
  // Logitech c270 camera
  // f = (640 pixels)*(4 mm)/(
  param.calib.f  = /*679.0;*/645.24;//707.1; // focal length in pixels
  param.calib.cu = /*659.8;*/661.96;//601.9; // principal point (u-coordinate) in pixels
  param.calib.cv = /*186.57;*/194.13;//183.11; // principal point (v-coordinate) in pixels
  param.height = 1.0; // m
  
  // init visual odometry
  VisualOdometryMono viso(param);
  
  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);
  //vector<Matcher::p_match> p_matched_;
  
  //cv::namedWindow("image",0);

  ofstream fichier("resultat_pfe_hole_img.txt", ios::out | ios::trunc);
  if(!fichier) {
	 cerr << "Impossible d'ouvrir le fichier !" << endl;
  }

  /*char base[256]; sprintf(base,"%06d.png",0);
  string img_file_name  = dir + "/I1_" + base;
  Mat img = cv::imread(img_file_name,0);
  imshow("image",img);
  waitKey(1);
  detector->detect(img, prev_keypoint);
  extractor->compute(img, prev_keypoint, prev_descriptor);*/
    
  // loop through all frames i=0:372
  for (int32_t i=0; i</*373*1106*/1022; i++) {

    // input file names
    char base_name[256]; sprintf(base_name,"%06d.png",i);
    string left_img_file_name  = dir + "/I1_" + base_name;
    //string left_img_file_name  = dir + "/" + base_name;
    
    // catch image read/write errors here
    try {

      // load left and right input image
      cv::Mat left_img = cv::imread(left_img_file_name,0); // load as black and white image
      //left_img = left_img(Rect(306, 100, 612, 190));
      //left_img = left_img(Rect(306, 0, 612, 370));

      cv::imshow("image",left_img);
      cv::waitKey(1);
      // image dimensions
      int32_t dims[3];
      dims[0] = left_img.cols; // width
      dims[1] = left_img.rows; // height
      dims[2] = dims[0]; // image width, BW so it equals to width

      // get pointers to the image data
      uint8_t* left_img_data  = left_img.data;
      //p_matched_.clear();
      //feature_matching(left_img, p_matched_);

      // status
      cout << "Processing: Frame: " << i;
      
      // compute visual odometry
      if (viso.process(left_img, left_img_data, dims)) {

        // on success, update current pose
        pose = pose * Matrix::inv(viso.getMotion());
      
        // output some statistics
        double num_matches = viso.getNumberOfMatches();
        double num_inliers = viso.getNumberOfInliers();
        cout << ", Matches: " << num_matches;
        cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        cout << pose << endl;
        fichier << pose.val[0][3] << " " << pose.val[2][3] << endl;
	show_trajectory(pose);
      }
      else {
	   if(i==0) fichier << pose.val[0][3] << " " << pose.val[2][3] << endl;
	   cout << " ... failed!";
      }
      cout << endl;
    }
    // catch image read errors here
    catch (...) {
      cerr << "ERROR: Couldn't read input files!" << endl;
      return 1;
    }
  }
  
  // output
  cout << "Demo complete! Exiting ..." << endl;
  fichier.close();

  // exit
  return 0;
}
