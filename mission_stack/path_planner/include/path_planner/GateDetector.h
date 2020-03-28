#include <opencv2/opencv.hpp>
#include <iostream>
#include <limits.h>

using namespace cv;
using namespace std;

/**

  * \brief the below class contains all members and functions for detecting gate and its related parameters

*/
class GateDetector
{
public:

  /**
   constructor
   */
    GateDetector();

    /**
     destructor
    */
	~GateDetector();

  /**

    * \brief function for detecting gate and returns true or false based on the same

    * \param[in] orig input feed

  */
    bool detectGate(Mat orig);
    double dist(Point p1, Point p2);

    int nCenters, nCentersIndex;
    vector<Point> lastCenters;
    Point gateCenter; //!< gate center coordinates
    Rect gateRect; //!< rectangle containing the gate
    int scale, delta, ddepth; //!< respective parameters
    int sobelThresholdX, sobelThresholdY, morph_size; //!< sobel parameters
    double max_cols = 648, max_rows = 488;
    Mat src, temp, cdst;
    Mat grad_x,grad_y,src_gray;
    Mat abs_grad_x,abs_grad_y;

};
