#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace cv;
using namespace std;

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::system_clock::time_point TimePoint;

/**

 * \brief an enum type for different states for performing the bins task

*/
enum state {DONOTHING,
            FIRSTBINSEARCH,
            HEIGHTADJUST,
            RIGHTBINSEARCH,
            LEFTBINSEARCH,
            DESCENDTODROPBALL,
            EXITAFTERDROPBALL,
            ENTERTOPICKBALL,
            FINDDROPBIN,
            PICKUPBALL};

/**

 * \brief an enum type for color of the bin

*/
enum bincolor {RED,BLUE,UNKNOWN};

/**

 * \brief BinsTask class contains all the members and functions for performing the bins task

*/
class BinsTask
{
public:

  /**
     constructor
  */
    BinsTask();

  /**
     destructor
  */
    ~BinsTask();

/**

 * \brief function for finding out and return the time difference between the start and end of a sub-task

 * \param[in] TimePoint(1) stores end time of the sub-task

 * \param[in] TimePoint(2) stores start time of the sub-task

 */
    double findTimeDifference(TimePoint , TimePoint);

/**

 * \brief function coordinating all the sub-tasks , returns bool values

 * \param[in] src input feed

 * \param[in] controlSetpoint a Vec3i pointer for contolling the setpoints

 * \param[in] timeout a bool pointer to ensure efficiency in sub-tasks

*/
    bool doTasks(Mat src, Vec3i &controlSetpoint,bool &timeout);

/**

  * \brief function for setting the states for corresponding sub-task , returns bool values

  * \param[in] orig cloned image of input feed

  * \param[in] controlSetpoint a Vec3i pointer for controlling the setpoints

  * \param[in] currentstate stores the state of the current sub-task performed

*/
    bool setState(Mat orig, Vec3i &controlSetpoint, state currentstate);

/**

 * \brief function for resetting all parameters for performing sub-tasks

*/
    void reset();

/**

 * \brief function for searching out the first bin and returns a counter for identified bins

 * \param[in] confidenceCtr the returning counter

 * \param[in] orig cloned image of input feed

 * \param[in] dispimg diaplaying the image with identified bin

 * \param[in] motionTarget a Vec3i pointer for storing the center of the rectangle of identifed bins

*/
    int firstbinsearch(int confidenceCtr, Mat orig, Mat dispimg,  Vec3i &motionTarget);

    /**

     * \brief function for height adjustment and returns a counter for the identified bins

     * \param[in] confidenceCtr the returning counter

     * \param[in] orig cloned image of input feed

     * \param[in] dispimg diaplaying the image with identified bin

     * \param[in] motionTarget a Vec3i pointer for storing the center of the rectangle of identifed bins

    */
    int heightAdjust(int confidenceCtr, Mat orig,Mat dispimg,  Vec3i &motionTarget);

    /**

     * \brief function for searching out the next right bin and returns a counter for identified bins

     * \param[in] confidenceCtr the returning counter

     * \param[in] orig cloned image of input feed

     * \param[in] dispimg diaplaying the image with identified bin

     * \param[in] crop a Rect type pointer storing the cropped rectangle coordinates

     * \param[in] motionTarget a Vec3i pointer for storing the center of the rectangle of identifed bins

    */
    int rightbinsearch(int confidenceCtr, Mat orig, Mat dispimg, Rect &crop, Vec3i &motionTarget);

    /**

     * \brief function for searching out the next left bin and returns a counter for identified bins

     * \param[in] confidenceCtr the returning counter

     * \param[in] orig cloned image of input feed

     * \param[in] dispimg diaplaying the image with identified bin

     * \param[in] crop a Rect type pointer storing the cropped rectangle coordinates

     * \param[in] motionTarget a Vec3i pointer for storing the center of the rectangle of identifed bins

    */
    int leftbinsearch(int confidenceCtr,Mat orig, Mat dispimg, Rect &crop,  Vec3i &motionTarget);

    /**

     * \brief function for descending the bot to drop the ball in the bin and returns a counter for identified bins

     * \param[in] confidenceCtr the returning counter

     * \param[in] orig cloned image of input feed

     * \param[in] dispimg diaplaying the image with identified bin

     * \param[in] motionTarget a Vec3i pointer for storing the center of the rectangle of identifed bins

    */
    int descendtodropball(int confidenceCtr, Mat orig,Mat dispimg,  Vec3i &motionTarget);

    /**

     * \brief function for finding the bin where the ball will be dropped and returns a counter for identified bins

     * \param[in] confidenceCtr the returning counter

     * \param[in] orig cloned image of input feed

     * \param[in] dispimg diaplaying the image with identified bin

     * \param[in] motionTarget a Vec3i pointer for storing the center of the rectangle of identifed bins

    */
    int finddropbin(int confidenceCtr, Mat orig, Mat dispimg,  Vec3i &motionTarget);

    /**

     * \brief function for finding the bin center and returns the centre point of the bin

     * \param[in] boundBox stores the rectangle which contains the identified bin

    */
    Point findBinCenter(Rect boundBox);

    /**

     * \brief function for finding blue bin and returns a rectangle containing the same

     * \param[in] orig cloned image of input feed

     * \param[in] useMatSurrounding decides whether to use mat surrounding or not

     */
    Rect findBluebin(Mat orig,bool useMatSurrounding);

    /**

      * \brief function for finding red bin and returns a rectangle containing the same

      * \param[in] orig cloned image of input feed

      * \param[in] useMatSurrounding decides whether to use mat surrounding or not

    */
    Rect findRedbin(Mat orig,bool useMatSurrounding);

    /**

       * \brief function for segmenting out red bins and returns the thresholded one of the same

       * \param[in] img hsv converted image of orig

     */
    Mat segmentRed(Mat img);

    /**

      * \brief function for segmenting out blue bin and returns the thresholded one of the same

      * \param[in] img hsv converted image of orig

    */
    Mat segmentBlue(Mat img);

    /**

      * \brief function for segmenting out green mat and returns the thresholded one of the same

      * \param[in] img hsv converted image of orig

    */
    Mat segmentGreen(Mat img);

    TimePoint startTime; //!< stores the start time of sub-tasks
    bincolor  currentBin; //!< stores the current bin color

    state currentstate; //!< stores the state based on current sub-task
    int timelimit; //!< stores time limit of each sub-task
    bool stateComplete; //!< true - sub-task complete , false - sub-task not complete
    bool stateTransition; //!< true - sub-task changed successfully , false - sub-task cannot be changed
    vector<state> stateList; //!< stores all sub-tasks state in a sequence
    vector<int> stateTimeouts; //!< stores all sub-tasks timeouts in sequence
    unsigned int stateCounter; //!< counter storing no. of completed states

    Mat orig, maindisp; //!< stores cloned image of src and sub-task oriented image respectively
};
