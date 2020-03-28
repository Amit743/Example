#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

/**

  * \brief the below class contains all the variables and functions for yaw contol

  */
class YawControl
{
public:

  /**

   * \brief parameterised constructor ; initializes canny parameters and yawFramesRequired

   * \param[in] int_type stores frame count

  */
    YawControl(int);

    /**
       destructor
       */
    ~YawControl();

    /**

     * \brief function clears all yaw control sub-tasks

     */
    void reset();

    /**

     * \brief function for finding the median of yaw angle and
              returns a array of half the input yaw angle array size storing the median values

     * \param[in] values an array containing the yaw angles

     */
    double findMedian(vector<double> values);

    /**

     * \brief function returns filtered indexes array containing median angles

     * \param[in] angles stores the raw yaw values

    */
    vector<int> filterOutliers(vector<Vec4i> lines, vector<double> angles);

    /**

    	* \brief function determines the current head direction from initial to determine the yaw

      * \param[in] orig input feed

    */
    bool findHeadingDiff(Mat orig);
    Mat src, dst, cdst, orig;
    int canny_low, canny_high, thresholdVal, minLineLength, maxLineGap; //!< canny parameters
    vector<double> yawVec; //!< array storing yaw velocities
    int ind;
    double headingDiff; //<! stores rotation angle difference
    int yawFramesRequired; //<! stores required no. of frames for determing the yaw
};
