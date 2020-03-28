#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

/**

 * \brief AlignBin class contains all the variables and functions for alligning bin task

*/

class AlignBin
{
public:

  /**
     constructor
     */
    AlignBin();

    /**
       destructor
       */
    ~AlignBin();

/**

  * \brief function for identifying out bins from the input feed , returns true or false

  * \param[in] Mat for image type input

  * \param[in] bool for returning true / false

*/
    bool findBins(Mat, bool);

/**

 * \brief function for selecting a bin from the identified ones , returns true or false

 * \param[in] lBins stores the 8-bit single-channel image to be labeled

 * \param[in] labelsBins destination labeled image

 * \param[in] statsBins contains the information about the labeled bins

 * \param[in] cropRect contains the cropped out rectangle which contains the identified bin

 * \param[in] max_area_index a pointer storing the index for the area of the rectangle taken into consideration

 */
    bool findBinToChoose(int lBins, Mat labelsBins, Mat statsBins, Rect cropRect, int &max_area_index);

/**

 * \brief function for updating the inner crop of the bins identified

 * \param[in] statsBins contains the information about the labeled bins

 * \param[in] index stores the serial no. of the labelled image in use

*/
    void updateInnerCrop(Mat statsBins, int index);

/**

 * \brief function for initializing the inner crop of the identified bins

 * \param[in] statsBins contains the information about the labeled bins

 * \param[in] index stores the serial no. of the labelled image in use

*/
    void initializeInnerCrop(Mat statsBins, int index);

    double max_cols = 648;
    double max_rows = 488;

    double BIN_INNER_CROP_RATIO = 2;
    double BIN_AREA_THRESHOLD = 0.022;
    double BIN_REQUIRED_RATIO = 4;

    double bins_offset_x = 0.2;
    double bins_offset_y = 0.1;

    Rect bin_outer_crop = Rect(Point(bins_offset_x*max_cols,0.4*max_rows),
                Point((1.0-bins_offset_x)*max_cols,(1.0-bins_offset_y)*max_rows)); //!< drawing out a rectangle for outer cropping of the bin

    bool bin_found = false;
    Rect bin_inner_crop; //!< rectangle for inner crop of the identified bin

    int bin_not_found_ctr, bin_area_exceeded_ctr, bin_found_confidence; //!< parameters for correct identification of bin

    // first set H - hue , S - saturation , V - value , second letters H - high , L - low
    int HL=25,HH=155;
    int SL1=95, SH1=255;
    int VL1=0,VH1=150;

    // second set - full hue and sat
    int VL2=0,VH2=70;
    int SL2=0, SH2=255;

    bool last_bin_found = false;

    Point binCenter; //!< stores the identified bin center
    Mat draw, thresh; //!< Mat type for drawing rectangles over identified bins and thresholding respectively 
};
