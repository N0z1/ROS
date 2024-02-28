#ifndef _MY_BLOBS_H_
#define _MY_BLOBS_H_
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

int MyConnectedComponentsWithStats(const Mat &bin_img, Mat &label_img, vector<int>&areas, vector<Rect>&locations, vector<Point2f>&centroids, vector<int>&labels, bool showLabelled=false);

#endif
