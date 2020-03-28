#include "path_planner/PathPlanner.h"

#define binOnly 0

PathPlanner::PathPlanner(ros::NodeHandle _nh, int nextSwayMode, int swayRange):nh(_nh),it(_nh)
{
    cout<<"Next mode: "<<nextSwayMode<<endl;
    cout<<"Sway range: "<<swayRange<<endl;

    for(int i = 0;i < 6; i++)
    {
        sp.setpoints.push_back(0);
    }
    //sp.setpoints[0] = 3;
    sp.setpoints[2] = 1004;
    sp.setpoints[4] = 5.7;

    frontCameraSub = nh.subscribe("/front_camera/image_rect_color",1,&PathPlanner::frontCameraCallback, this);
    bottomCameraSub = nh.subscribe("/bottom_camera/image_rect_color",1,&PathPlanner::bottomCameraCallback, this);
    vectornavSub = nh.subscribe("/vectornav",1,&PathPlanner::vectornavCallback, this);

    frontDebugPub = it.advertise("/front_camera/debug", 1);
    bottomDebugPub = it.advertise("/bottom_camera/debug", 1);

    setpointPub = nh.advertise<pid_controller::Setpoint>("/setpoints", 1);
    drop_pub = nh.advertise<std_msgs::Int8>("/actuator/ball_drop", 1);
    breachPub = nh.advertise<std_msgs::Bool>("/breach_command",1);
    descend_pub = nh.advertise<std_msgs::Int8>("/actuator/picker_descend", 1);
    ascend_pub = nh.advertise<std_msgs::Int8>("/actuator/picker_ascend", 1);

    findGateOb = new FindGate(nextSwayMode,swayRange);
    gateDetectorOb = new GateDetector();
    yawControlOb = new YawControl(CORRECT_YAW_FRAMES);
    alignBinOb = new AlignBin();
    binsTaskOb = new BinsTask();

    gateFrameCount = 0;

    gateSearchStarted = false;
    gateFound = false;
    gateAligned = false;
    gateReached = false;
    gateFinalYawAligned = false;
    gateFinalAligned = false;
    gateCrossed = false;
#if binOnly
    returnToCenter = true;
#else
    returnToCenter = false;
#endif
    foundBin = false;
    reachedBin = false;
    alignedWithBin = false;
    binTaskComplete = false;
    gateDetectedLast = false;

    rightSwayUnits = 0;
    swayCheckTime = Clock::now();
    lastYawCorrectTime = Clock::now();
    yawChangedLast = Clock::now();
    pauseYawCorrection = false;

    stateList.push_back(FIRSTBINSEARCH); stateTimeouts.push_back(120); tasks.push_back(NONE);
    stateList.push_back(DESCENDTODROPBALL); stateTimeouts.push_back(60); tasks.push_back(DROP);
    stateList.push_back(HEIGHTADJUST); stateTimeouts.push_back(60); tasks.push_back(NONE);
    stateList.push_back(EXITAFTERDROPBALL); stateTimeouts.push_back(15); tasks.push_back(NONE);
    stateList.push_back(FIRSTBINSEARCH); stateTimeouts.push_back(120); tasks.push_back(NONE);
    stateList.push_back(DESCENDTODROPBALL); stateTimeouts.push_back(60); tasks.push_back(RELEASEPICKER);
    stateList.push_back(HEIGHTADJUST); stateTimeouts.push_back(60); tasks.push_back(NONE);
    stateList.push_back(EXITAFTERDROPBALL); stateTimeouts.push_back(15); tasks.push_back(NONE);

    binsTaskOb->stateList = stateList;
    binsTaskOb->stateTimeouts = stateTimeouts;
    binTaskTimeout = false;
    heightChangedLast = Clock::now();
    goingForwardToSearch = false;
    currentYaw = 0;storedYaw = 0;
    start_1 = Clock::now();
    start_2 = Clock::now();
    
}

PathPlanner::~PathPlanner(){}

double PathPlanner::findTimeDifference(TimePoint end, TimePoint start)
{
    return ((double)std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count())*0.001;
}

void PathPlanner::findSetpoints()
{
#if !binOnly
    if(!gateFound)
    {   
            if(findTimeDifference(Clock::now(),start_1) > 1){
                
                if(sp.setpoints[2] < 1015)
                sp.setpoints[2] = sp.setpoints[2] + 1;

                // cout<<"Depth: "<<sp.setpoints[2];
                publishSetpoints();
                start_1 = Clock::now();
            }
        
        findGateOb->findNext();
        if(findGateOb->searchStarted) // Check if search is to be started
        {
            if(!gateSearchStarted)
            {
        cout<<"Gate search started"<<endl;
                gateFrameCount = 0;
                gateSearchStarted = true;
            }
        }

        if(gateFrameCount == MAX_FIND_GATE_FRAMES) // Check if search completed
        {
            if(gateFound)
                return;

            gateSearchStarted = false;

            findGateOb->searchStarted = false;
            findGateOb->searchComplete = true;
        }

        sp.setpoints[0] = findGateOb->forwardSpeed;
        //sp.setpoints[2] = 1010;
        sp.setpoints[1] = findGateOb->sideSpeed;
    }

    if(gateFound && !gateAligned)
    {   if(findTimeDifference(Clock::now(),start_2) > 1){
                
                if(sp.setpoints[2] < 1020)
                sp.setpoints[2] = sp.setpoints[2] + 1;

                // cout<<"Depth: "<<sp.setpoints[2];
                publishSetpoints();
                start_2 = Clock::now();
            }
        
        if(gateDetectedLast)
        {
            prevCommandTime = Clock::now();
            double pos, width, err;
            pos = gateDetectorOb->gateCenter.x;
            width = gateDetectorOb->gateRect.width;
            if(abs(err = (imageWidth/2 - pos)) > 0.23*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 10;
                else sp.setpoints[1] = -15;
            }
            else if(abs(err) > 0.1*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 5;
                else sp.setpoints[1] = -10;
            }
            else if(abs(err) > 0.02*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 3;
                else sp.setpoints[1] = -5;
            }
            else
            {
                gateAligned = true;
                cout<<"Gate Aligned"<<endl;
            }
        }
        else if(findTimeDifference(Clock::now(),prevCommandTime) > 0.5)
        {
            sp.setpoints[0] = 0;
            sp.setpoints[1] = 0;
        }

        if(findTimeDifference(Clock::now(),lastFrontCameraCallbackTime) > 1)
            gateDetectedLast = false;
    }

    if(gateAligned && !gateReached)
    {   sp.setpoints[2] = 1020;
        if(gateDetectedLast)
        {
            prevCommandTime = Clock::now();
            double pos, width, height, err;
            pos = gateDetectorOb->gateCenter.x;
            width = gateDetectorOb->gateRect.width;
            height = gateDetectorOb->gateRect.height;

            if(abs(err = (imageWidth/2 - pos)) > 0.23*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 10;
                else sp.setpoints[1] = -15;
            }
            else if(abs(err) > 0.1*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 5;
                else sp.setpoints[1] = -10;
            }
            else if(abs(err) > 0.02*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 3;
                else sp.setpoints[1] = -5;
            }
            else
            {
                sp.setpoints[0] = 15;
                sp.setpoints[1] = 0;
            }

            double area = width * height;
            if(area/imgArea > 0.22)
            {
                cout<<"Gate reached"<<endl;
                gateReached = true;
            }
        }
        else if(findTimeDifference(Clock::now(),prevCommandTime) > 0.5)
        {
            sp.setpoints[0] = 0;
            sp.setpoints[1] = 0;
        }
        if(findTimeDifference(Clock::now(),lastFrontCameraCallbackTime) > 1)
            gateDetectedLast = false;
    }

    if(gateReached && !gateFinalAligned)
    {
        cout<<"IN final align"<<endl;
        if(gateDetectedLast)
        {
            prevCommandTime = Clock::now();
            double pos, width, height, err;
            pos = gateDetectorOb->gateCenter.x;
            width = gateDetectorOb->gateRect.width;
            height = gateDetectorOb->gateRect.height;

            if(abs(err = (imageWidth/2 - pos)) > 0.23*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 10;
                else sp.setpoints[1] = -15;
            }
            else if(abs(err) > 0.1*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 5;
                else sp.setpoints[1] = -10;
            }
            else if(abs(err) > 0.02*width )
            {
                sp.setpoints[0] = 2;

                if(err < 0) sp.setpoints[1] = 3;
                else sp.setpoints[1] = -5;
            }
            else
            {
                cout<<"Gate final Aligned"<<endl;
                gateFinalAligned = true;
                gateCrossTimer = Clock::now();
            }
        }
        else if(findTimeDifference(Clock::now(),prevCommandTime) > 0.5)
        {
            sp.setpoints[0] = 0;
            sp.setpoints[1] = 0;
        }
        if(findTimeDifference(Clock::now(),lastFrontCameraCallbackTime) > 1)
            gateDetectedLast = false;
    }
    if(gateFinalAligned && !gateCrossed)
    {
        if(findTimeDifference(Clock::now(),gateCrossTimer) < GATE_CROSS_TIME)
        {
            sp.setpoints[0] = 15;
            sp.setpoints[1] = 0;
        }
        else
        {
            gateCrossed = true;
            cout<<"Gate crossed!"<<endl;
            swayReturnTimer = Clock::now();
        }
    }
    if(gateCrossed && !returnToCenter)
    {
        if(findTimeDifference(Clock::now(),swayReturnTimer) < abs(rightSwayUnits/10.0))
        {
            sp.setpoints[0] = 0;
            if(rightSwayUnits > 0)
            {
                sp.setpoints[1] = -10;
            }
            else
            {
                sp.setpoints[1] = 10;
            }
        }
        else
        {
            sp.setpoints[0] = 0;
            sp.setpoints[1] = 0;
            returnToCenter = true;
            pauseYawCorrection = true;
            storedYaw = currentYaw;
        }
    }
#endif
    if(returnToCenter && !foundBin)
    {   
        sp.setpoints[2] = 1020;
        pauseYawCorrection = true;

        if(goingForwardToSearch)
        {
            if(findTimeDifference(Clock::now(),forwardSearch) < FORWARD_SEARCH_TIME)
            {
                sp.setpoints[0] = 10;
                sp.setpoints[1] = 0;
            }
            else
            {
                goingForwardToSearch = false;
                sp.setpoints[0] = 0;
                sp.setpoints[1] = 0;
                storedYaw = currentYaw;
                goingForwardCount ++;
                cout<<"Forward complete"<<endl;
            }
            return;
        }


                goingForwardToSearch = true;
                forwardSearch = Clock::now();
                cout<<"Going forward to search"<<endl;

        // if(findTimeDifference(Clock::now(),yawChangedLast) > 1)
        // {
        //     sp.setpoints[5] = storedYaw + yawChange;
        //     yawChangedLast = Clock::now();
        //     cout<<"New Yaw: "<<sp.setpoints[5]<<endl;
        //     if(yawChange >= BASE_YAW_SEARCH_ANGLE + goingForwardCount*5)
        //     {
        //         yawChangeDir = -1;
        //     }
        //     if(yawChange <= - BASE_YAW_SEARCH_ANGLE - goingForwardCount*5)
        //     {
        //         yawChangeDir = +1;
        //         yawChange = 0;
        //         goingForwardToSearch = true;
        //         forwardSearch = Clock::now();
        //         sp.setpoints[5] = storedYaw;
        //         cout<<"Going forward to search"<<endl;
        //     }
        //     yawChange = yawChange + yawChangeDir*yawJump;
        // }
    }
    if(foundBin && !reachedBin)
    {
        if(alignBinOb->last_bin_found)
        {
            bin_not_found_ctr = 0;
            double pos, err;
            pos = alignBinOb->binCenter.x;

            if(abs(err = (imageWidth/2 - pos)) > 0.2*imageWidth )
            {
                sp.setpoints[0] = 0;

                if(err < 0) sp.setpoints[1] = 10;
                else sp.setpoints[1] = -10;

                cout<<"foundBin setpoints[1] = 10"<<endl;
            }
            else if(abs(err) > 0.05*imageWidth )
            {
                sp.setpoints[0] = 0;

                if(err < 0) sp.setpoints[1] = 5;
                else sp.setpoints[1] = -5;

                cout<<"foundBin setpoints[1] = 5"<<endl;
            }
            else if(abs(err) > 0.01*imageWidth )
            {
                sp.setpoints[0] = 0;

                if(err < 0) sp.setpoints[1] = 3;
                else sp.setpoints[1] = -3;

                cout<<"foundBin setpoints[1] = 3"<<endl;
            }
            else
            {
                sp.setpoints[0] = 10;
                sp.setpoints[1] = 0;
            }
        }
        else
        {
            bin_not_found_ctr++;
            if(bin_not_found_ctr>10)
                foundBin = false;
            sp.setpoints[0] = 0;
            sp.setpoints[1] = 0;
        }
    }
    if(reachedBin && !alignedWithBin)
    {
        if(alignBinOb->last_bin_found)
        {
            double pos, err;
            pos = alignBinOb->binCenter.x;

            if(abs(err = (imageWidth/2 - pos)) > 0.2*imageWidth )
            {
                sp.setpoints[0] = 0;

                if(err < 0) sp.setpoints[1] = 5;
                else sp.setpoints[1] = -5;

                cout<<"reachedBin setpoints[1] = 5"<<endl;
            }
            else if(abs(err) > 0.05*imageWidth )
            {
                sp.setpoints[0] = 0;

                if(err < 0) sp.setpoints[1] = 3;
                else sp.setpoints[1] = -3;

                cout<<"reachedBin setpoints[1] = 3"<<endl;
            }
            else if(abs(err) > 0.01*imageWidth )
            {
                sp.setpoints[0] = 0;

                if(err < 0) sp.setpoints[1] = 2;
                else sp.setpoints[1] = -2;

                cout<<"reachedBin setpoints[1] = 2"<<endl;
            }
            else
            {
                alignedWithBin = true;
                sp.setpoints[0] = 0;
                sp.setpoints[1] = 0;
                sp.setpoints[2] = 1020;
            }
        }
        else
        {
            sp.setpoints[0] = 0;
            sp.setpoints[1] = 0;
        }
    }
    if(alignedWithBin && !binTaskComplete)
    {
        int posX, posY;
        int desiredX = imageHeight/2, desiredY = imageWidth/2;

        posY = binsSetpoint[0];
        posX = binsSetpoint[1];

        int errorX, errorY;

        if(abs(errorX = (desiredX - posX)) > 0.2*imageWidth)
        {
            if(errorX > 0) sp.setpoints[0] = 5;
            else sp.setpoints[0] = -5;

            cout<<"alignedWithBin setpoints[0] = 10"<<endl;
        }
        else if(abs(errorX) > 0.1*imageWidth)
        {
            if(errorX > 0) sp.setpoints[0] = 3;
            else sp.setpoints[0] = -3;

            cout<<"alignedWithBin setpoints[0] = 5"<<endl;
        }
        else if(abs(errorX) > 0.001*imageWidth)
        {
            if(errorX > 0) sp.setpoints[0] = 2;
            else sp.setpoints[0] = -2;

            cout<<"alignedWithBin setpoints[0] = 3"<<endl;
        }
        else
        {
            sp.setpoints[0] = 0;
        }

        if(abs(errorY = (desiredY - posY)) > 0.4*imageWidth)
        {
            if(errorY < 0) sp.setpoints[1] = 5;
            else sp.setpoints[1] = -5;
            cout<<"else alignedWithBin setpoints[1] = 5"<<endl;
        }
        else if(abs(errorY) > 0.1*imageWidth)
        {
            if(errorY < 0) sp.setpoints[1] = 3;
            else sp.setpoints[1] = -3;
            cout<<"else alignedWithBin setpoints[1] = 3"<<endl;
        }
        else if(abs(errorY) > 0.001*imageWidth)
        {
            if(errorY < 0) sp.setpoints[1] = 1;
            else sp.setpoints[1] = -1;
            cout<<"else alignedWithBin setpoints[1] = 2"<<endl;
        }
        else
        {
            sp.setpoints[1] = 0;
        }
        
        if(binsSetpoint[2]!=0 && findTimeDifference(Clock::now(),heightChangedLast) > 0.1)
        {
            cout<<"Trying to change height binsSetpoint[2]-> "<<binsSetpoint[2]<<endl;
            heightChangedLast = Clock::now();
            sp.setpoints[2] = sp.setpoints[2] + binsSetpoint[2];
            if (sp.setpoints[2]>1025)
            {
                sp.setpoints[2] = 1025;
            }
        }
    }
    if(binTaskComplete)
    {
        breachPub.publish(breachMsg);
        sp.setpoints[0] = 0;
        sp.setpoints[1] = 0;
    }
}

void PathPlanner::publishSetpoints()
{
    if(!gateCrossed)
    {
        rightSwayUnits += (sp.setpoints[1] * findTimeDifference(Clock::now(),swayCheckTime));
        swayCheckTime = Clock::now();
    }
    setpointPub.publish(sp);
}

void PathPlanner::frontCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    lastFrontCameraCallbackTime = Clock::now();

    Mat orig;

    try
    {
        if(!cv_bridge::toCvShare(msg, "bgr8")->image.empty())
        {
            orig=cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        else
        {
            AUV_ERROR("Image is empty");
            sp.setpoints[0] = sp.setpoints[1] = 0;
            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        AUV_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        sp.setpoints[0] = sp.setpoints[1] = 0;
        return;
    }
#if !binOnly
    if(!gateSearchStarted)
    return;

    if(gateFrameCount < MAX_FIND_GATE_FRAMES && !gateFound)
    {
        Mat mask = Mat::zeros(imageHeight, imageWidth, CV_8UC3); // all 0
        mask(Rect(0.05*imageWidth,0.1*imageHeight,0.9*imageWidth,0.8*imageHeight)) = Scalar(255,255,255);
        Mat roiOrig;
        bitwise_and(orig,mask,roiOrig);
        gateFound = gateDetectorOb->detectGate(orig);
        if(gateFound)
            cout<<"Found gate"<<endl;
        frontDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", gateDetectorOb->cdst).toImageMsg();
        frontDebugPub.publish(frontDebugMsg);
        gateFrameCount++;
        return;
    }
    if(!gateReached)
    {
        gateDetectedLast = gateDetectorOb->detectGate(orig);
        frontDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", gateDetectorOb->cdst).toImageMsg();
        frontDebugPub.publish(frontDebugMsg);
        return;
    }
    if(!gateFinalAligned)
    {
        gateDetectedLast = gateDetectorOb->detectGate(orig);
        frontDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", gateDetectorOb->cdst).toImageMsg();
        frontDebugPub.publish(frontDebugMsg);
        return;
    }
#endif
    if(returnToCenter && !foundBin)
    {
        foundBin = alignBinOb->findBins(orig,true);
        if(foundBin)
        {
            sp.setpoints[0] = 0;
            sp.setpoints[1] = 0;
        }
        frontDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", alignBinOb->draw).toImageMsg();
        frontDebugPub.publish(frontDebugMsg);
        return;
    }
    if(foundBin && !reachedBin)
    {
        reachedBin = alignBinOb->findBins(orig,false);
        if(reachedBin)
        {
            sp.setpoints[0] = 0;
            sp.setpoints[1] = 0;
        }
        frontDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", alignBinOb->draw).toImageMsg();
        frontDebugPub.publish(frontDebugMsg);
        return;
    }
    if(reachedBin && !alignedWithBin)
    {
        alignBinOb->findBins(orig,false);
        frontDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", alignBinOb->draw).toImageMsg();
        frontDebugPub.publish(frontDebugMsg);
        return;
    }
}


void PathPlanner::bottomCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat orig;

    try
    {
        if(!cv_bridge::toCvShare(msg, "bgr8")->image.empty())
        {
            orig=cv_bridge::toCvShare(msg, "bgr8")->image;
            flip(orig,orig,-1);
        }
        else
        {
            AUV_ERROR("Image is empty");
            sp.setpoints[0] = sp.setpoints[1] = 0;
            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        AUV_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        sp.setpoints[0] = sp.setpoints[1] = 0;
        return;
    }

    if(alignedWithBin && !binTaskComplete)
    {
        binTaskComplete = binsTaskOb->doTasks(orig,binsSetpoint,binTaskTimeout);
        if(binsTaskOb->stateComplete)
        {
            if(tasks[binsTaskOb->stateCounter] == DROP)
            {
                cout << "Drop the ball" << endl;
                ballDropSequence();
            }
            if(tasks[binsTaskOb->stateCounter] == RELEASEPICKER)
            {
                cout << "Picking the ball" << endl;
                ballPickupSequence();
            }
        }
        bottomDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", binsTaskOb->maindisp).toImageMsg();
        bottomDebugPub.publish(bottomDebugMsg);
    }
    if(USE_YAW_CORRECTION)
    {
        bool correctedGateYawAvailable = yawControlOb->findHeadingDiff(orig);
        if(!pauseYawCorrection && correctedGateYawAvailable && findTimeDifference(Clock::now(),lastYawCorrectTime) > 5)
        {
            headingDiff = yawControlOb->headingDiff;
            cout<<"Yaw should be: "<<currentYaw + headingDiff<<endl;
            sp.setpoints[5] = currentYaw + headingDiff;
            yawControlOb->reset();
            lastYawCorrectTime = Clock::now();
        }
        bottomDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", yawControlOb->cdst).toImageMsg();
        bottomDebugPub.publish(bottomDebugMsg);
    }
}

void PathPlanner::vectornavCallback(const vectornav::VectorNavData::ConstPtr &msg)
{
    currentYaw = msg->orientation[2];
}

void PathPlanner::ballDropSequence()
{
    dropMsg.data = 2;
    for(int i=0;i<5;i++)
        drop_pub.publish(dropMsg);
}

void PathPlanner::ballPickupSequence()
{
    sp.setpoints[0] = 3;
    publishSetpoints();

    TimePoint start = Clock::now();
    while(findTimeDifference(Clock::now(),start) < 2){
        publishSetpoints();
    }

    sp.setpoints[0] = 0;
    sp.setpoints[2] = sp.setpoints[2] + 1;
    publishSetpoints();

    for(int i=0;i<5;i++){

        descend_pub.publish(descendMsg);
        publishSetpoints();
    }
    start = Clock::now();
    while(findTimeDifference(Clock::now(),start) < 10){
        publishSetpoints();
    }

    storedYaw = currentYaw;
    for(int i=1;i<360;i++)
    {
        sp.setpoints[5] = storedYaw + i;
        if(sp.setpoints[5] > 180)
            sp.setpoints[5] -= 360;
        publishSetpoints();
        start = Clock::now();
        while(findTimeDifference(Clock::now(),start) < 0.06){
            publishSetpoints();
        }
    }

    sp.setpoints[5] = storedYaw;
    sp.setpoints[2] = 1004;
    publishSetpoints();

    for(int i=0;i<5;i++){

        ascend_pub.publish(ascendMsg);
        publishSetpoints();
    }

    start = Clock::now();
    while(findTimeDifference(Clock::now(),start) < 10){
        publishSetpoints();
    }

}
