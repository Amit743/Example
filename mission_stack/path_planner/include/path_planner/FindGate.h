#include <iostream>
#include <chrono>

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::system_clock::time_point TimePoint;

using namespace std;

/**

 * \brief the below class contains all variables and functions for finding the gate

*/
class FindGate
{
public:

  /**

   * \brief parameterised constructor ; initialises all the int and bool parameters of the class

   * \param[in] int(1) next mode of the auv

   * \param[in] int(2) sway range

   */
    FindGate(int,int);

    /**
     destructor
    */
	~FindGate();

  /**

   * \brief function for finding out time difference between auv moving commands for finding the gate and returns the same

   * \param[in] TimePoint(1) current clock time

   * \param[in] TimePoint(2) start time of auv command

   */
    double findTimeDifference(TimePoint , TimePoint );

#if 1
    const int INITIAL_DEPTH_TIME = 2;
    const int INITIAL_FORWARD_TIME = 20; // 15
    const int SWAY_WAIT_TIME = 3;
    const int SWAY_TIME = 10;
    int SWAY_MAX = 5;

    // Debug values
#else
    const int INITIAL_DEPTH_TIME = 0;
    const int INITIAL_FORWARD_TIME = 0;
    const int SWAY_WAIT_TIME = 0;
    const int SWAY_TIME = 0;
    int SWAY_MAX = 5;
#endif

    TimePoint startTime;

    int currentMode, nextMode; // -1 -> left, 0 -> center, 1 -> right

    bool initialDepthStarted, initialDepthComplete;
    bool initialForwardStarted, initialForwardComplete;
    bool waitBeforeSwayStarted, waitBeforeSwayCompleted;
    bool swayStarted, swayCompleted;

    bool readyToStartSearch, searchStarted, searchComplete;

    int forwardSpeed, sideSpeed;

  /**

     * \brief function which checks the corresponding auv command and executes it accordingly

  */
    void findNext();
};
