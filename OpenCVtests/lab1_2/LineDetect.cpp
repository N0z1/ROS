#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
 if (argc < 4)
 {
  printf("This program demonstrates line finding with the Hough transform.\n");
  printf("Usage:\n  ./LineDetect <image_name> low_thr high_thr [optional hough threshold]\n");
  return -1;
 }
 const char* filename = argv[1];
 Mat src = imread(filename, 1);
 if(src.empty())
 {
     printf("Can not open %s", filename);
     return -1;
 }
 int low_theshold = atoi(argv[2]);
 int high_theshold = atoi(argv[3]);
 if(low_theshold >= high_theshold)
 {
     printf("The lower threshold needs to be smaller than the upper threshold\n");
     return -1;
 }

 Mat grey, edges;
 cvtColor(src, grey, CV_BGR2GRAY); // Canny works on greyscale images
 // CANNY EDGE DETECTOR
 Canny(src, edges, low_theshold, high_theshold);
 imshow("source", src);
 imshow("Edges", edges);
 waitKey(0);

 //exit(0); //STEP 1 - REMOVE LATER TO CONTINUE THE PROGRAM

 // STANDARD HOUGH TRANSFORM
 // 'Vec2f var;'  is the same as 'float var[2];'
 // It will contain 2 folating point values (the rho and theta coordinates of the lines)
 // 'vector<type> var;' creates a C++ dynamic array of the indicated type
 vector<Vec2f> lines, Vlines;
 // HoughLines(InputArray image, OutputArray lines, double rho, double theta, int threshold);
 // CHOOSE THRESHOLD TO GET BETTER RESULTS
 int Hough_theshold = 150;
 if(argc == 5)  Hough_theshold = atoi(argv[4]);
 HoughLines(edges, lines, 1, CV_PI/180, Hough_theshold); //size_t is unsigned long int
 printf("Standard Hough found %ld lines\n",lines.size());
 // Draw the lines extracted
 Mat coloredges;
 cvtColor(edges, coloredges, CV_GRAY2BGR);
 for( size_t i = 0; i < lines.size(); i++ ) //'size_t' is 'unsigned long int'
 {
     Vec2f l = lines[i];
     float rho = l[0], theta = l[1];
	 if(1) //UPDATE THIS CONDITION TO SELECT ONLY VERTICAL LINES
	 {	 
	  Vlines.push_back(l);
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line( coloredges, pt1, pt2, Scalar(255,0,0), 1, CV_AA);
	 }
 }
 printf("Selected %ld lines\n",Vlines.size());
 imshow("Selected Hough lines", coloredges);
 waitKey(0);

 return 0;
}

