#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include <cv.h>
#include <math.h>


using namespace cv;
using namespace std;


#define MAXTIME 10.

void showImage(string name,Mat img, bool print=0, string path=""){
    cv::namedWindow(name, WINDOW_NORMAL);
    cv::imshow(name, img);
    cv::resizeWindow(name, 800,800);
    if(print == 1){
        path.append(name);
        imwrite(path, img);
    }

}

bool containedIn( Vec2f coordinate ,vector<Vec2f> vec){
    for(size_t i = 0 ; i < vec.size(); i++){
        if(vec[i][0] == coordinate[0] && vec[i][1] == coordinate[1]){
            return true;
        }
    }
    return false;
}

vector<Vec2f> hough2Cart(Vec2f hough){
    vector<Vec2f> outputVec;
    float rho = hough[0];
    float theta = hough[1];
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;

    //Define points of the line
    outputVec.push_back(Vec2f(cvRound(x0 + 5000*(-b)), cvRound(y0 + 5000*(a))));//Extending the line from top to :Angle*5000pixels + offset
    outputVec.push_back(Vec2f(cvRound(x0 - 5000*(-b)), cvRound(y0 - 5000*(a))));
    return outputVec;
}

void drawHoughLines(vector<Vec2f> houghEdges, Mat houghImg){
    //Used for drawing the lines https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
    for( size_t i = 0; i < houghEdges.size(); i++ )
    {
    //Draw houghLines
    float rho = houghEdges[i][0];
    float theta = houghEdges[i][1];
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;

    //Define points of the line
    Point pt1(cvRound(x0 + 5000*(-b)),      //Extending the line from top to :Angle*5000pixels + offset
              cvRound(y0 + 5000*(a)));
    Point pt2(cvRound(x0 - 5000*(-b)),
              cvRound(y0 - 5000*(a)));

    //Draw line
    line( houghImg, pt1, pt2, Scalar(255,255,255), 1, 8 );
    }
}

void drawCircles(vector<Vec2i> coordinateVec, Mat img){
    for(size_t i = 0; i < coordinateVec.size(); i++)
        circle(img,Point(coordinateVec[i][1],coordinateVec[i][0]),5,Scalar(0,0,0),2);
}

bool intersection(Vec2f line1,Vec2f line2, Vec2i &dstVector){
    vector<Vec2f> line1Coordinates = hough2Cart(line1);
    vector<Vec2f> line2Coordinates = hough2Cart(line2);

    float y1 = line1Coordinates[0][0];
    float x1 = line1Coordinates[0][1];
    float y2 = line1Coordinates[1][0];
    float x2 = line1Coordinates[1][1];

    float y3 = line2Coordinates[0][0];
    float x3 = line2Coordinates[0][1];
    float y4 = line2Coordinates[1][0];
    float x4 = line2Coordinates[1][1];

    float numX = (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4);
    float denumX = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);

    float numY = (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4);
    float denumY = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);

    /*cout << "numX: " << numX << endl;
    cout << "numY: " << numY << endl;
    cout << "denumX: " << denumX << endl;
    cout << "denumY: " << denumY << endl;
*/
    if(denumX==0 || denumY==0){
        return false;
    }
    else{
        //calc the intersection coordinates, if there are one
        dstVector[0] = cvRound(numX/denumX);
        dstVector[1] = cvRound(numY/denumY);

        /*
        cout << "Xval: " << dstVector[0] << endl;
        cout << "Yval: " << dstVector[1] << endl;
        */
        //return true if there was a intersection, false of not
        return true;
    }


}

int main(int argc, char** argv) {
    Mat img1 = cv::imread("/home/kasper/RWworkspace/markerImages/sequence_2a_h/marker_thinline_hard_05.png", CV_LOAD_IMAGE_GRAYSCALE);
    Mat img = cv::imread("/home/kasper/RWworkspace/markerImages/sequence_2a_h/marker_thinline_hard_05.png", CV_LOAD_IMAGE_GRAYSCALE);
    showImage("imageTest", img1);
    //Average filter
    Mat kernel = Mat::ones(5,5,CV_8U);
    blur(img1,img1,kernel.size());

    //create edges
    Mat edges;
    Canny(img1,edges,100,180);
    showImage("edges", edges);

    //Hough transform on the edges
    vector<Vec2f> houghEdges;
    HoughLines(edges,houghEdges,1,CV_PI/180,110);

    vector<Vec2f> perpEdges;
    double perpThreshold = 0.1; //threshold for difference in angle from 90 degrees
    Mat houghImg = img1;
    Mat perpImg = img;

    //extract line which is perpendicular to each other
    //hough sorts the lines in the number of votes of a descending order
    //houghEdges[i][1] (0 ~ vertical lines) (pi/2~horizontal line)
    for( size_t i = 0; i < houghEdges.size(); i++ )
    {
        double upperThresh = houghEdges[i][1] + perpThreshold;
        double lowerThresh = houghEdges[i][1] - perpThreshold;
        for(size_t j = i; j <houghEdges.size(); j ++){
            if(upperThresh + CV_PI/2> houghEdges[j][1] && lowerThresh + CV_PI/2 < houghEdges[j][1] && i!=j){ // add line to perpEdges if difference is bewteen +- ½Pi+threshold
                perpEdges.push_back(houghEdges[i]);
            }
            if(upperThresh - CV_PI/2 > houghEdges[j][1] && lowerThresh - CV_PI/2 < houghEdges[j][1] && i!=j ){// add line to perpEdges if difference is bewteen +- ½Pi+threshold
                perpEdges.push_back(houghEdges[i]);
            }
        }
    }

    //remove duplicate lines
    vector<Vec2f> uniquePerpEdges;
    for(size_t i = perpEdges.size()-1; i < perpEdges.size(); i--){
        if(!containedIn(perpEdges[i],uniquePerpEdges)){
            uniquePerpEdges.push_back(perpEdges[i]);
        }
    }

    //Determine which lines to keep by thresholding for
    //how many intersections each line has with other lines e.g. line-line intersection
    //https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#In_two_dimensions
    vector<Vec2i> intersectionVec;
    Vec2i coordinateVec;
    int intersectThresh = 8;
    int voteThresh = 4;
    vector<int> voteVector;
    vector<Vec2i> neighbourCoor;
    //---------------------------------------TBD-------------------------------------//

    //creating a intersection vector
    for(size_t i = 0; i < uniquePerpEdges.size(); i++){
        for(size_t j = i; j < uniquePerpEdges.size(); j++){
            if(intersection(uniquePerpEdges[i], uniquePerpEdges[j],coordinateVec)==true){
                if(coordinateVec[1] < img1.cols && coordinateVec[0] < img1.rows){
                    intersectionVec.push_back(coordinateVec);   //push back the intersection coordiante onto intersectionVec
                }

            }
        }
    }

    //Get the votes for how many "neighbours" the intersections has
    int vote = 0;
    for(size_t i = 0; i < intersectionVec.size(); i++){
        for(size_t j = 0; j < intersectionVec.size(); j++){
            if(intersectionVec[i][0] + intersectThresh > intersectionVec[j][0] && intersectionVec[i][0] - intersectThresh < intersectionVec[j][0]){ //threshold x-val
                if(intersectionVec[i][1] + intersectThresh > intersectionVec[j][1] && intersectionVec[i][1] - intersectThresh < intersectionVec[j][1]){ //threshold y-val
                    vote++;
                }
            }
        }
    voteVector.push_back(vote); //pushback the votes of the vector
    vote = 0;
    }

    //Extract the coordinates with most votes
    for(size_t i = 0; i < intersectionVec.size(); i++){
        if(voteVector[i] >= voteThresh){
            neighbourCoor.push_back(intersectionVec[i]);
        }
    }

    //---------------------------------------TBDEND----------------------------------//

    //Draws hough lines onto img
    drawHoughLines(houghEdges,houghImg);

    //Draw perpendicular lines extracted onto perpImg
    drawHoughLines(uniquePerpEdges,perpImg);

    //Draw the coordinates from the intersections
    drawCircles(neighbourCoor,perpImg);

    cout << "img size: " << houghImg.size() << endl ;
    cout << "Hough Edges: " << houghEdges.size() << endl;
    cout << "perpendicular Edges:"<< perpEdges.size() << endl;
    cout << "Unique Edges: " << uniquePerpEdges.size() << endl;
    cout << "intersections: " << intersectionVec.size() << endl;
    cout << "voteVector Length: " << voteVector.size() << endl;
    cout << "neighbouring threshold intersections: " << neighbourCoor.size() << endl;

    imshow("houghEdges", houghImg);
    imshow("uniquePerpEdges", perpImg);



    while (cv::waitKey() != 27); // (d½o nothing)

    return 0;
}
