#ifndef SEQ2_HPP
#define SEQ2_HPP
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include <cv.h>
#include <math.h>
#include <cmath>
#include <string>

using namespace std;
using namespace cv;

void showImage(string name,Mat img, bool print=0, string path="");

bool containedIn( Vec2f coordinate ,vector<Vec2f> vec);
bool containedInThresh( Vec2i coordinate ,vector<Vec2i> vec, int thresh);
vector<Vec2f> hough2Cart(Vec2f hough);
void drawHoughLines(vector<Vec2f> houghEdges, Mat houghImg);
void drawCircles(vector<Vec2i> coordinateVec, Mat img, Scalar color);
bool intersection(Vec2f line1,Vec2f line2, Vec2i &dstVector);
void seq2Algo(string imagePath, string pathToWrite);
Mat getMask(Mat img,Scalar minimumHSV, Scalar maxHSV);
Vec2i seq1Algo(Mat img1, string pathToWrite = "");

#endif // SEQ2_HPP
