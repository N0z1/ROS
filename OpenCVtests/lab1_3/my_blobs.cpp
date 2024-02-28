#include "my_blobs.h"

int MyConnectedComponentsWithStats(const Mat &bin_img, Mat &label_img, vector<int>&areas, vector<Rect>&locations, vector<Point2f>&centroids, vector<int>&labels, bool showLabelled)
{
    int x, y, i, j, n, area, sumx, sumy;
    std::vector < std::vector<cv::Point2i > > blobs;
    Point2f cdg;
    blobs.clear();
    areas.clear();
    locations.clear();
    centroids.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground
    bin_img.convertTo(label_img, CV_32SC1,1.0/255.0); //map 255 to 1 on int image
    int label_count = 2; //label 1 means unprocessed pixel, 2 is first meaningful label
    for(y=0; y < label_img.rows; y++) {
        int *row = (int*)label_img.ptr(y);
        for(x=0; x < label_img.cols; x++) {
            if(row[x] != 1) {
                continue;
            }

            Rect rect;
            floodFill(label_img, Point(x,y), label_count, &rect, 0, 0, 4);
            vector <Point2i> blob;
	    area=0; sumx=0; sumy=0;
            for(i=rect.y; i < (rect.y+rect.height); i++)
	    {
                int *row2 = (int*)label_img.ptr(i);
                for(j=rect.x; j < (rect.x+rect.width); j++)
		 {
                    if(row2[j] != label_count) {
                        continue;
                    }
		    area++; sumx += j; sumy += i;
                    blob.push_back(cv::Point2i(j,i));
                 }
            }
	    labels.push_back(label_count);
	    locations.push_back(rect);
            blobs.push_back(blob);
	    areas.push_back(area);
	    cdg.x = (float)(sumx/(float)area);
	    cdg.y = (float)(sumy/(float)area);
            centroids.push_back(cdg);
            label_count++;
        }
    }

    n=blobs.size();

    if(showLabelled)
    {
     cout<<"Found "<<blobs.size()<<" blobs\n";
     namedWindow("Labelling", WINDOW_AUTOSIZE);
     Mat output = cv::Mat::zeros(bin_img.size(), CV_8UC3);
     for(size_t i=0; i < blobs.size(); i++) {
        unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
        unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
        unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));

        for(size_t j=0; j < blobs[i].size(); j++) {
            int x = blobs[i][j].x;
            int y = blobs[i][j].y;

            output.at<cv::Vec3b>(y,x)[0] = b;
            output.at<cv::Vec3b>(y,x)[1] = g;
            output.at<cv::Vec3b>(y,x)[2] = r;
        }
     }
     imshow("Labelling", output);
     waitKey(0);
     destroyWindow("Labelling");
    }
    
    return n;
}

