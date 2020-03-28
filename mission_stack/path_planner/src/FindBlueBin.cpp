#include "path_planner/FindBlueBin.h"

FindBlueBin::FindBlueBin(int next, int swayRange)
{
    SWAY_MAX = swayRange;
    nextMode = next;
    currentMode = 0;

    forwardSpeed = 0;
    sideSpeed = 0;

    initialDepthStarted = true;
    initialDepthComplete = true;
    initialForwardStarted = false;
    initialForwardComplete = false;
    waitBeforeSwayStarted = false;
    waitBeforeSwayCompleted = false;
    swayStarted = false;
    swayCompleted = false;

    readyToStartSearch = true;
    searchStarted = false;
    searchComplete = false;
}

FindBlueBin::~FindBlueBin()
{

}

double FindBlueBin::findTimeDifference(TimePoint end, TimePoint start)
{
    return ((double)std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count())*0.001;
}
void FindBlueBin::findNext()
{
    // Give some time to attain depth initially
    if(!initialDepthStarted)
    {
        startTime = Clock::now();
        initialDepthStarted = true;
        cout<<"Starting Inital depth"<<endl;
    }
    if(!initialDepthComplete && (findTimeDifference(Clock::now(),startTime)  < INITIAL_DEPTH_TIME))
    {
        forwardSpeed = 0;
        sideSpeed = 0;
        return;
    }
    if(!initialDepthComplete)
        cout<<"Inital depth complete"<<endl;
    initialDepthComplete = true;

    // go forward for some time initially
    if(!initialForwardStarted)
    {
        startTime = Clock::now();
        initialForwardStarted = true;
        cout<<"Starting Inital forward"<<endl;
    }
    if(!initialForwardComplete &&  (findTimeDifference(Clock::now(),startTime)  < INITIAL_FORWARD_TIME))
    {
        forwardSpeed = 15;
        sideSpeed = 0;
        return;
    }
    if(!initialForwardComplete)
        cout<<"Inital forward complete"<<endl;
    initialForwardComplete = true;

    // start sway motion indefinitely

    // wait before starting next sway
    if(!waitBeforeSwayStarted)
    {
        startTime = Clock::now();
        waitBeforeSwayStarted = true;
        cout<<"Wait Before Sway Started"<<endl;
    }
    if(!waitBeforeSwayCompleted && (findTimeDifference(Clock::now(),startTime)  < SWAY_WAIT_TIME))
    {
        forwardSpeed = 0;
        sideSpeed = 0;
        return;
    }
    if(!waitBeforeSwayCompleted)
        cout<<"Wait Before Sway Completed"<<endl;
    waitBeforeSwayCompleted = true;

    // inform that it is safe to start searching
    if(!searchStarted && readyToStartSearch)
    {
        cout<<"Now starting search"<<endl;
        readyToStartSearch = false;
        searchStarted = true;
        searchComplete = false;
        return;
    }
    if(!searchComplete)
    {
        return;
    }

    if(!swayStarted)
    {
        startTime = Clock::now();
        swayStarted = true;
        cout<<"Sway started: "<<currentMode<<" -> "<<nextMode<<endl;
    }
    if(!swayCompleted && (findTimeDifference(Clock::now(),startTime) < SWAY_TIME))
    {
        forwardSpeed = 0;
        if(nextMode - currentMode > 0)
            sideSpeed = 10;
        else
            sideSpeed = -10;
        return;

    }
    if(!swayCompleted)
        cout<<"Sway completed"<<endl;
    swayCompleted = true;

    if(swayCompleted)
    {
        if (nextMode == SWAY_MAX)
        {
            // bot was swaying from center to right
            currentMode = nextMode;
            nextMode = currentMode - 1;
        }
        else if (nextMode == -SWAY_MAX)
        {
            // bot was swaying from center to left
            currentMode = -SWAY_MAX;
            nextMode = currentMode + 1;
        }
        else
        {
            if(nextMode - currentMode > 0)
            {
                currentMode = nextMode;
                nextMode = currentMode + 1;
            }
            else
            {
                currentMode = nextMode;
                nextMode = currentMode - 1;
            }
        }

        waitBeforeSwayStarted = false;
        waitBeforeSwayCompleted = false;
        swayStarted = false;
        swayCompleted = false;

        readyToStartSearch = true;
        searchStarted = false;
        searchComplete = false;

        forwardSpeed = 0;
        sideSpeed = 0;
        return;
    }

}
