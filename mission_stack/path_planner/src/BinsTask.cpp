#include "path_planner/BinsTask.h"

int confidenceCtr = 0;
double max_rows = 488;
double max_cols = 648;
int centerSlack = max_cols*0.1;

#define HEIGHTADJUST_FAR   0.3
#define HEIGHTADJUST_CLOSE 0.7
double SIDECROP_PERCENT = (1.0-HEIGHTADJUST_CLOSE)/4;
#define HEIGHTADJUST_DESCEND 0.8
#define BBOX_FILLRATIO 0.3

#define SEARCHBLUE

Rect cropRect;
Mat elementSmall = getStructuringElement( MORPH_ELLIPSE, Size( 3,3 ));
Mat elementLarge = getStructuringElement( MORPH_ELLIPSE, Size( 5,5 ));
Mat elementExtremelyLarge = getStructuringElement( MORPH_ELLIPSE, Size( 7,7 )); // Le y hutiya name

BinsTask::BinsTask()
{
    reset();
}

BinsTask::~BinsTask()
{}

void BinsTask::reset()
{
    currentstate = DONOTHING;
    stateComplete = true;
    stateList.clear();
    stateTimeouts.clear();
    stateCounter = -1;
    currentBin = UNKNOWN;
}

double BinsTask::findTimeDifference(TimePoint end, TimePoint start)
{
    return ((double)std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count())*1e-3;
}

Mat BinsTask::segmentRed(Mat img)
{
    // Use HSV channels Hue and Saturation
    // Probably use dilate here itself
    Mat thresh1,thresh2,thresh;
    inRange(img,Scalar(0,100,0),Scalar(11,255,255),thresh1);
    inRange(img,Scalar(170,100,0),Scalar(180,255,255),thresh2);
    bitwise_or(thresh1,thresh2,thresh);
    dilate(thresh,thresh,elementSmall);
    return thresh;
}

Mat BinsTask::segmentGreen(Mat img)
{
    // Use HSV channels Hue and Saturation
    // Probably use dilate here itself
    Mat thresh;
    inRange(img,Scalar(35,0,0),Scalar(79,255,255),thresh);
    erode(thresh,thresh,elementLarge);
    dilate(thresh,thresh,elementExtremelyLarge);
    return thresh;
}

Mat BinsTask::segmentBlue(Mat img)
{
    // Use HSV channels Hue and Saturation
    // Probably use dilate here itself
    Mat thresh;
    inRange(img,Scalar(82,0,0),Scalar(111,255,255),thresh);
    dilate(thresh,thresh,elementSmall);
    return thresh;
}

Rect BinsTask::findRedbin(Mat orig, bool useMatSurrounding = false)
{
    // convert to HSV -- or pass HSV directly
    // segment image for red
    Mat hsv;
    cvtColor(orig,hsv,CV_BGR2HSV);
    Mat redSegment_greenCrop;
    Rect greenBox;
    Mat redSegment = segmentRed(hsv);
    //imshow("Segment Red",redSegment);
    // segment image for green
    if(useMatSurrounding)
    {
        Mat greenSegment = segmentGreen(hsv);
        //imshow("Segment Green",greenSegment);
        Mat labelsG,statsG,centroidsG;
        int lG = connectedComponentsWithStats(greenSegment, labelsG, statsG, centroidsG);
        if(lG <= 1)
            return Rect(0,0,0,0);
        int large_ind_G=-1;
        int max_size_G=0;
        // Find connectedcomponent with largest area
        for(int i = 1; i <lG; i++)
        {
			// Cannot fill percentage reject on green mat
			//if( (statsG.at<int>(i,CC_STAT_HEIGHT)*statsG.at<int>(i,CC_STAT_WIDTH)) * BBOX_FILLRATIO > statsG.at<int>(i,CC_STAT_AREA) )
			//	continue;
			if(max_size_G<(statsG.at<int>(i,CC_STAT_AREA)))
            {
				max_size_G = statsG.at<int>(i,CC_STAT_AREA);
                large_ind_G = i;
            }
        }
        if(large_ind_G == -1)
            return Rect(0,0,0,0);
        greenBox.y = statsG.at<int>(large_ind_G,CC_STAT_TOP);
        greenBox.x = statsG.at<int>(large_ind_G,CC_STAT_LEFT);
        greenBox.height = statsG.at<int>(large_ind_G,CC_STAT_HEIGHT);
        greenBox.width  = statsG.at<int>(large_ind_G,CC_STAT_WIDTH);

        redSegment_greenCrop = redSegment(greenBox);
    }
    else
        redSegment_greenCrop = redSegment;
    ////imshow("Redboxcheck ",redSegment_greenCrop);
    // Crop the greenbox and find the red inside it
    // Might cause problem if bin is too big in the image
    Mat labelsR,statsR,centroidsR;

    int lR = connectedComponentsWithStats(redSegment_greenCrop, labelsR, statsR, centroidsR);
    if(lR <= 1)
        return Rect(0,0,0,0);
    int large_ind_R=-1;
    int max_size_R=0;
    // Find connectedcomponent with largest area
    for(int i = 1; i <lR; i++)
    {
		if( (statsR.at<int>(i,CC_STAT_HEIGHT)*statsR.at<int>(i,CC_STAT_WIDTH)) * BBOX_FILLRATIO > statsR.at<int>(i,CC_STAT_AREA) )
				continue;
        if(max_size_R<(statsR.at<int>(i,CC_STAT_AREA)))
        {
            max_size_R = statsR.at<int>(i,CC_STAT_AREA);
            large_ind_R = i;
        }
    }
    if(large_ind_R == -1)
        return Rect(0,0,0,0);
    Rect redBox;
    if(useMatSurrounding)
    {
        redBox.y = statsR.at<int>(large_ind_R,CC_STAT_TOP) + greenBox.y;
        redBox.x = statsR.at<int>(large_ind_R,CC_STAT_LEFT) + greenBox.x;
    }
    else
    {
        redBox.y = statsR.at<int>(large_ind_R,CC_STAT_TOP);
        redBox.x = statsR.at<int>(large_ind_R,CC_STAT_LEFT);
    }
    //
    redBox.height = statsR.at<int>(large_ind_R,CC_STAT_HEIGHT);
    redBox.width  = statsR.at<int>(large_ind_R,CC_STAT_WIDTH);

    return redBox;
}

Rect BinsTask::findBluebin(Mat orig, bool useMatSurrounding = true)
{
    // convert to HSV -- or pass HSV directly
    // segment image for blue
    Mat hsv;
    Mat blueSegment_greenCrop;
    Rect greenBox;
    cvtColor(orig,hsv,CV_BGR2HSV);
    Mat blueSegment = segmentBlue(hsv);
    //imshow("Blue Segmented",blueSegment);
    if(useMatSurrounding)
    {
        // segment image for green
        Mat greenSegment = segmentGreen(hsv);
        //imshow("Segment Green",greenSegment);
        Mat labelsG,statsG,centroidsG;
        int lG = connectedComponentsWithStats(greenSegment, labelsG, statsG, centroidsG);
        if(lG <= 1)
            return Rect(0,0,0,0);
        int large_ind_G=-1;
        int max_size_G=0;
        // Find connectedcomponent with largest area
        for(int i = 1; i <lG; i++)
        {
			// Cannot fill percentage reject on green mat
			//if( (statsG.at<int>(i,CC_STAT_HEIGHT)*statsG.at<int>(i,CC_STAT_WIDTH)) * BBOX_FILLRATIO > statsG.at<int>(i,CC_STAT_AREA) )
			//	continue;
			if(max_size_G<(statsG.at<int>(i,CC_STAT_AREA)))
			{
				max_size_G = statsG.at<int>(i,CC_STAT_AREA);
                large_ind_G = i;
            }
        }
        if(large_ind_G == -1)
            return Rect(0,0,0,0);
        greenBox.y = statsG.at<int>(large_ind_G,CC_STAT_TOP);
        greenBox.x = statsG.at<int>(large_ind_G,CC_STAT_LEFT);
        greenBox.height = statsG.at<int>(large_ind_G,CC_STAT_HEIGHT);
        greenBox.width  = statsG.at<int>(large_ind_G,CC_STAT_WIDTH);

        blueSegment_greenCrop = blueSegment(greenBox);
    }
    else
    {
        blueSegment_greenCrop = blueSegment;
    }
    // Crop the greenbox and find the blue inside it
    // Might cause problem if bin is too big in the image
    Mat labelsR,statsR,centroidsR;

    int lR = connectedComponentsWithStats(blueSegment_greenCrop, labelsR, statsR, centroidsR);
    if(lR <= 1)
        return Rect(0,0,0,0);
    int large_ind_R=-1;
    int max_size_R=0;
    // Find connectedcomponent with largest area
    for(int i = 1; i <lR; i++)
    {
		if( (statsR.at<int>(i,CC_STAT_HEIGHT)*statsR.at<int>(i,CC_STAT_WIDTH)) * BBOX_FILLRATIO > statsR.at<int>(i,CC_STAT_AREA) )
				continue;
		if(max_size_R<(statsR.at<int>(i,CC_STAT_AREA)))
        {
			max_size_R = statsR.at<int>(i,CC_STAT_AREA);
            large_ind_R = i;
        }
    }
    if(large_ind_R == -1)
        return Rect(0,0,0,0);
    Rect blueBox;
    if(useMatSurrounding)
    {
        blueBox.y = statsR.at<int>(large_ind_R,CC_STAT_TOP) + greenBox.y;
        blueBox.x = statsR.at<int>(large_ind_R,CC_STAT_LEFT) + greenBox.x;
    }
    else
    {
        blueBox.y = statsR.at<int>(large_ind_R,CC_STAT_TOP);
        blueBox.x = statsR.at<int>(large_ind_R,CC_STAT_LEFT);
    }
    blueBox.height = statsR.at<int>(large_ind_R,CC_STAT_HEIGHT);
    blueBox.width  = statsR.at<int>(large_ind_R,CC_STAT_WIDTH);

    return blueBox;
}

Point BinsTask::findBinCenter(Rect boundBox)
{
    if(boundBox.height > max_rows*0.01 && boundBox.width > max_cols*0.01 &&
        (boundBox.height * boundBox.width) > (max_rows*max_cols)*0.05)	// Size rejection
	{
        Point boundCenter;
        boundCenter.x = boundBox.x + boundBox.width/2;
        boundCenter.y = boundBox.y + boundBox.height/2;
		// If close to center :  increase the size rejection limit
		if(boundCenter.x < max_cols/2.0 + centerSlack*2 && boundCenter.x > max_cols/2.0 - centerSlack*2 &&
            boundCenter.y < max_rows*(2/3.0) + centerSlack*2 && boundCenter.y > max_rows*(2/3.0) - centerSlack*2)
		{
			if(!(boundBox.height > max_rows*0.1 && boundBox.width > max_cols*0.1 &&
				(boundBox.height * boundBox.width) > (max_rows*max_cols)*0.15))
					return Point(-1,-1);
		}

        return boundCenter;
    }
    return Point(-1,-1);
}

/*bool BinsTask::atGreenMatEdge(Mat orig,Point matDirection) // not ready yet
{
    Mat greenThresh = segmentGreen(orig);
    Mat quad_tl = greenThresh(Rect(0,0,max_cols/2,max_rows/2)) & Rect(0,0,max_cols,max_rows);
    Mat quad_tr = greenThresh(Rect(max_cols/2,0,max_cols/2,max_rows/2)) & Rect(0,0,max_cols,max_rows);
    Mat quad_bl = greenThresh(Rect(0,max_rows/2,max_cols/2,max_rows/2)) & Rect(0,0,max_cols,max_rows);
    Mat quad_br = greenThresh(Rect(max_cols/2,max_rows/2,max_cols/2,max_rows/2)) & Rect(0,0,max_cols,max_rows);

    double tlp,trp,blp,brp;
    double totalPixels = max_rows*max_cols;
    tlp = countNonZero(quad_tl)/totalPixels;
    trp = countNonZero(quad_tr)/totalPixels;
    blp = countNonZero(quad_bl)/totalPixels;
    brp = countNonZero(quad_br)/totalPixels;
}
*/

int BinsTask::firstbinsearch(int confidenceCtr, Mat orig, Mat dispimg,  Vec3i &motionTarget)
{
    Rect redBox =  findRedbin(orig);
    Point redCenter = findBinCenter(redBox);

    if(redBox.width >0 && redBox.height >0 && redCenter.x != -1 && redCenter.y != -1)
        {
        rectangle( dispimg, redBox, Scalar(0,0,255), 2, 8, 0 );
        circle(dispimg,redCenter,5,Scalar(255,0,0),2,5,0);
        ////imshow("Red bin",dispimg);
        if(redCenter.x < max_cols/2.0 + centerSlack && redCenter.x > max_cols/2.0 - centerSlack &&
            redCenter.y < max_rows/2.0 + centerSlack && redCenter.y > max_rows/2.0 - centerSlack)
        {
            // Motion Control : Halt
            motionTarget[0] = redCenter.x;
            motionTarget[1] = redCenter.y;
            motionTarget[2] = 0;
            return confidenceCtr+1;
        }
        // Motion control : Set both forward and right
        motionTarget[0] = redCenter.x;
        motionTarget[1] = redCenter.y;
        motionTarget[2] = 0;
        return 0;
    }
    else
        return -1; // reset confidenceCtr
}

int BinsTask::heightAdjust(int confidenceCtr, Mat orig,Mat dispimg,  Vec3i &motionTarget)
{
    Rect binBox;
    Rect redBox =  findRedbin(orig);
#ifdef SEARCHBLUE
    Rect blueBox =  findBluebin(orig);
    if(blueBox.width * blueBox.height > redBox.width * redBox.height)
    {
        binBox = blueBox;
        currentBin = BLUE;
    }
    else
    {
        currentBin = RED;
        binBox = redBox;
    }
#else
    currentBin = RED;
    binBox = redBox;
#endif
    //cout<<"Bb"<<binBox<<endl;

    Point binCenter = findBinCenter(binBox);

    rectangle( dispimg, binBox, Scalar(0,0,255), 2, 8, 0 );
    circle(dispimg,binCenter,5,Scalar(255,0,0),2,5,0);

    if(binBox.width >0 && binBox.height >0 && binCenter.x != -1 && binCenter.y != -1)
    {
        if(binCenter.x < max_cols/2.0 + centerSlack && binCenter.x > max_cols/2.0 - centerSlack &&
            binCenter.y < max_rows/2.0 + centerSlack && binCenter.y > max_rows/2.0 - centerSlack)
        {
            cout<<double(binBox.width*1.0/max_cols)<<endl;
            if(binBox.width < max_cols*HEIGHTADJUST_FAR)
            {	// Motion control : Go down
                motionTarget[0] = binCenter.x;
                motionTarget[1] = binCenter.y;
                motionTarget[2] = 1;
            }
            else if(binBox.width > max_cols*HEIGHTADJUST_CLOSE)
            {	// Motion control : Go up
                motionTarget[0] = binCenter.x;
                motionTarget[1] = binCenter.y;
                motionTarget[2] = -1;
            }
            else
            {
                // Motion control : Halt
                motionTarget[0] = binCenter.x;
                motionTarget[1] = binCenter.y;
                motionTarget[2] = 0;
                binBox = binBox;
                return ++confidenceCtr;
            }
        }
        // Motion control : Set both forward and right according to center
            motionTarget[0] = binCenter.x;
            motionTarget[1] = binCenter.y;
            motionTarget[2] = 0;
        return 0;
    }
    else
        return -1; // reset confidenceCtr
}

int BinsTask::rightbinsearch(int confidenceCtr, Mat orig, Mat dispimg, Rect &crop, Vec3i &motionTarget)
{
    Mat searchcrop = orig(crop);
    Rect binBox;

    Rect redBox =  findRedbin(searchcrop);

#ifdef SEARCHBLUE
    Rect blueBox =  findBluebin(searchcrop);
    if(blueBox.width * blueBox.height > redBox.width * redBox.height)
    {
        binBox = blueBox;
        currentBin = BLUE;
    }
    else
    {
        binBox = redBox;
        currentBin = RED;
    }
#else
        currentBin = RED;
        binBox = redBox;
#endif
    // cout<<"Bb"<<binBox<<endl;

    Point binCenter = findBinCenter(binBox);
    Point binCenterGlobal;
    Rect newcrop;

    rectangle(dispimg,crop,Scalar(0,255,0),2,8,0);

    if(binBox.width >0 && binBox.height >0 && binCenter.x != -1 && binCenter.y != -1)
    {
        binCenterGlobal.x = crop.x + binCenter.x;
        binCenterGlobal.y = crop.y + binCenter.y;

        newcrop.x = 0.8*max_cols - (max_cols - (binBox.x + crop.x)) ;
        newcrop.y = 0;
        newcrop.height = max_rows;
        newcrop.width = 0.2*max_cols + (max_cols - (binBox.x + crop.x));
        newcrop = newcrop & Rect(0,0,max_cols,max_rows);
        crop = newcrop;
        circle(dispimg,binCenterGlobal,5,Scalar(0,255,0),2,5,0);
        //rectangle(dispimg,crop,Scalar(0,255,0),2,8,0);
        //cout<<binCenter<<endl;
        //cout<<crop<<endl;
        if(binCenterGlobal.x < max_cols/2.0 + centerSlack && binCenterGlobal.x > max_cols/2.0 - centerSlack &&
            binCenterGlobal.y < max_rows/2.0 + centerSlack && binCenterGlobal.y > max_rows/2.0 - centerSlack)
        {
            return confidenceCtr+1;
        }
         // Motion control : Set both forward and right according to center --> from binCenterGlobal
        motionTarget[0] = binCenterGlobal.x;
        motionTarget[1] = binCenterGlobal.y;
        motionTarget[2] = 0;
        return 0;
    }
    else
        return -1; // reset confidenceCtr
}

int BinsTask::leftbinsearch(int confidenceCtr,Mat orig, Mat dispimg, Rect &crop,  Vec3i &motionTarget)
{
    Mat searchcrop = orig(crop);
    Rect binBox;

    Rect redBox =  findRedbin(searchcrop);
#ifdef SEARCHBLUE

    Rect blueBox =  findBluebin(searchcrop);
    if(blueBox.width * blueBox.height > redBox.width * redBox.height)
    {
        binBox = blueBox;
        currentBin = BLUE;
    }
    else
    {
        currentBin = RED;
        binBox = redBox;
    }
#else
     currentBin = RED;
     binBox = redBox;
#endif// cout<<"Bb"<<binBox<<endl;

    Point binCenter = findBinCenter(binBox);
    Point binCenterGlobal;
    Rect newcrop;
    rectangle(dispimg,crop,Scalar(0,255,0),2,8,0);

    if(binBox.width >0 && binBox.height >0 && binCenter.x != -1 && binCenter.y != -1)
    {
        binCenterGlobal.x = binCenter.x;
        binCenterGlobal.y = binCenter.y;

        newcrop.x = 0;
        newcrop.y = 0;
        newcrop.height = max_rows;
        newcrop.width = binBox.x + binBox.width + 0.2*max_cols;
        newcrop = newcrop & Rect(0,0,max_cols,max_rows);
        crop = newcrop;
        circle(dispimg,binCenterGlobal,5,Scalar(255,0,0),2,5,0);
        //rectangle(dispimg,crop,Scalar(0,255,0),2,8,0);
        //cout<<binCenter<<endl;
        //cout<<crop<<endl;
        ////imshow("Found bin",orig);
        if(binCenterGlobal.x < max_cols/2.0 + centerSlack && binCenterGlobal.x > max_cols/2.0 - centerSlack &&
            binCenterGlobal.y < max_rows/2.0 + centerSlack && binCenterGlobal.y > max_rows/2.0 - centerSlack)
        {
            return confidenceCtr+1;
        }
        // Motion control : Set both forward and right according to center --> from binCenterGlobal
        motionTarget[0] = binCenterGlobal.x;
        motionTarget[1] = binCenterGlobal.y;
        motionTarget[2] = 0;
        return 0;
    }
    else
        return -1; // reset confidenceCtr
}

int BinsTask::descendtodropball(int confidenceCtr, Mat orig,Mat dispimg,  Vec3i &motionTarget)
{
    Rect binBox;
    Rect redBox =  findRedbin(orig);
#ifdef SEARCHBLUE
    Rect blueBox =  findBluebin(orig,false); // While descending cannot use the green mat for edges

    if(blueBox.width * blueBox.height > redBox.width * redBox.height)
    {
        binBox = blueBox;
        currentBin = BLUE;
    }
    else
    {
        currentBin = RED;
        binBox = redBox;
    }
#else
        currentBin = RED;
        binBox = redBox;
#endif
    // cout<<"Bb"<<binBox<<endl;

    Point binCenter = findBinCenter(binBox);

    rectangle( dispimg, binBox, Scalar(0,0,255), 2, 8, 0 );
    circle(dispimg,binCenter,5,Scalar(255,0,0),2,5,0);

    if(binBox.width >0 && binBox.height >0 && binCenter.x != -1 && binCenter.y != -1)
    {
        if(binCenter.x < max_cols/2.0 + centerSlack*2 && binCenter.x > max_cols/2.0 - centerSlack*2 &&
            binCenter.y < max_rows/2.0 + centerSlack*2 && binCenter.y > max_rows/2.0 - centerSlack*2)
        {
            // cout<<double(binBox.width*1.0/max_cols)<<endl;
            if(binBox.width < max_cols*HEIGHTADJUST_DESCEND)
            {	// Motion control : Go down
                motionTarget[0] = binCenter.x;
                motionTarget[1] = binCenter.y;
                motionTarget[2] = 1;   // 1 means go down
            }
            else
            {
                // Motion control : Halt
                motionTarget[0] = binCenter.x;
                motionTarget[1] = binCenter.y;
                motionTarget[2] = 0;
                binBox = binBox;
                return ++confidenceCtr;
            }
        }
        // Motion control : Set both forward and right according to center
        motionTarget[0] = binCenter.x;
        motionTarget[1] = binCenter.y;
        return 0;
    }
    else
        return -1; // reset confidenceCtr
}

int BinsTask::finddropbin(int confidenceCtr, Mat orig, Mat dispimg,  Vec3i &motionTarget)
{
    Rect redBox =  findRedbin(orig);
    Rect binBox;
    #ifdef SEARCHBLUE
        Rect blueBox =  findBluebin(orig,true);
        if(blueBox.width * blueBox.height > redBox.width * redBox.height)
        {
            currentBin = BLUE;
            binBox = blueBox;
        }
        else
        {
            currentBin = RED;
            binBox = redBox;
        }
    #else
        currentBin = RED;
        binBox = redBox;
    #endif

    Point binCenter = findBinCenter(binBox);


    if(binBox.width >0 && binBox.height >0 && binCenter.x != -1 && binCenter.y != -1)
    {
        rectangle( dispimg, binBox, Scalar(0,0,255), 2, 8, 0 );
        circle(dispimg,binCenter,5,Scalar(255,0,0),2,5,0);
        ////imshow("Drop bin",dispimg);
        if(binCenter.x < max_cols/2.0 + centerSlack && binCenter.x > max_cols/2.0 - centerSlack &&
            binCenter.y < max_rows/2.0 + centerSlack && binCenter.y > max_rows/2.0 - centerSlack)
        {
            // Motion Control : Halt
            motionTarget[0] = binCenter.x;
            motionTarget[1] = binCenter.y;
            motionTarget[2] = 0;
            return confidenceCtr+1;
        }
        // Motion control : Set both forward and right
        motionTarget[0] = binCenter.x;
        motionTarget[1] = binCenter.y;
        motionTarget[2] = 0;
        return 0;
    }
    else
        return -1; // reset confidenceCtr
}

bool BinsTask::setState(Mat orig, Vec3i &controlSetpoint, state currentstate)
{
    /*
        States:  ------- TODO: enum these in header
        DONOTHING
        FIRSTBINSEARCH
        HEIGHTADJUST
        RIGHTBINSEARCH
        LEFTBINSEARCH
        DESCENDTODROPBALL
        doabarrelroll_state
        EXITAFTERDROPBALL
        ENTERTOPICKBALL
        FINDDROPBIN
        PICKUPBALL
        abnachobenchod_state
    */

    maindisp = orig.clone();

    if(currentstate == FIRSTBINSEARCH)
    {
        if(stateTransition)
        {
            stateTransition = false;
            cout<<"Starting first bin search"<<endl;
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        confidenceCtr = firstbinsearch(confidenceCtr,orig,maindisp,controlSetpoint);
        if(confidenceCtr == -1)
        {  // Motion control : Default forward for this state
            controlSetpoint[0] = max_cols/2;
            controlSetpoint[1] = 0;
            controlSetpoint[2] = 0;
            cout<<"Not detected first bin"<<endl;
            confidenceCtr = 0;
        }
        else if(confidenceCtr > 10)
        {
            cout<<"First bin found."<<endl;
            return true;
        }
        else if(confidenceCtr!=0)
        {
            cout<<"First bin confidence: "<<confidenceCtr<<endl;
        }
        return false;
    }
    else if(currentstate == HEIGHTADJUST)
    {
        if(stateTransition)
        {
            cout<<"Starting height adjusting"<<endl;
            stateTransition = false;
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        confidenceCtr = heightAdjust(confidenceCtr,orig,maindisp,controlSetpoint);
        if(confidenceCtr == -1)
        {
            cout<<"Height not ready!"<<endl;
            confidenceCtr = 0;
        }
        else if(confidenceCtr > 10)
        {
            cout<<"Height adjust complete."<<endl;
            return true;
        }
        else if(confidenceCtr!=0)
        {
            cout<<"Height confidence: "<<confidenceCtr<<endl;
        }
        return false;
    }
    else if(currentstate == RIGHTBINSEARCH)
    {
        if(stateTransition)
        {
            cout<<"Starting right bin search"<<endl;
            stateTransition = false;
            int cropDim = (1.0-SIDECROP_PERCENT)*max_cols;   // TODO: Need macro here according to what heightadjust limit is set
            cropRect = Rect(cropDim,0,max_cols - cropDim,max_rows);
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        confidenceCtr = rightbinsearch(confidenceCtr,orig,maindisp,cropRect,controlSetpoint);
        if(confidenceCtr == -1)
        {
            controlSetpoint[0] = max_cols;
            controlSetpoint[1] = max_rows/2;
            controlSetpoint[2] = 0;

            cout<<"Right not found "<<endl;
            confidenceCtr = 0;
        }
        else if(confidenceCtr > 10)
        {
            cout<<"Right found. Done!"<<endl;
            return true;
        }
        else if(confidenceCtr!=0)
        {
            cout<<"Right bin confidence: "<<confidenceCtr<<endl;
        }
        return false;
    }
    else if(currentstate == LEFTBINSEARCH)
    {
        if(stateTransition)
        {
            cout<<"Starting left bin search"<<endl;
            stateTransition = false;
            int cropDim = SIDECROP_PERCENT*max_cols;
            cropRect = Rect(0,0,cropDim,max_rows);
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        confidenceCtr = leftbinsearch(confidenceCtr,orig,maindisp,cropRect,controlSetpoint);
        if(confidenceCtr == -1)
        {
            controlSetpoint[0] = 0;
            controlSetpoint[1] = max_rows/2;
            controlSetpoint[2] = 0;
            cout<<"Left not found "<<endl;
            confidenceCtr = 0;
        }
        else if(confidenceCtr > 10)
        {
            cout<<"Left found. Done!"<<endl;
            return true;
        }
        else if(confidenceCtr!=0)
        {
            cout<<"Left bin confidence: "<<confidenceCtr<<endl;
        }
        return false;
    }
    else if(currentstate == DESCENDTODROPBALL)
    {
        if(stateTransition)
        {
            cout<<"Starting descend to drop ball"<<endl;
            stateTransition = false;
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        confidenceCtr = descendtodropball(confidenceCtr,orig,maindisp,controlSetpoint);
        if(confidenceCtr == -1)
        {
            cout<<"Dafuq where to drop "<<endl;
            confidenceCtr = 0;
        }
        else if(confidenceCtr > 10)
        {
            cout<<"Daal do bin meh bc!"<<endl;
            return true;
        }
        else if(confidenceCtr!=0)
        {
            cout<<"Le waits to drop in bin, Confidence : "<<confidenceCtr<<endl;
        }
        return false;
    }
    else if(currentstate == EXITAFTERDROPBALL) // only backward and timeout for now
    {
        if(stateTransition)
        {
            cout<<"Exiting mat "<<endl;
            stateTransition = false;
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        // Motion control : Default backward for this state
        controlSetpoint[0] = max_cols/2;
        controlSetpoint[1] = max_rows;
        controlSetpoint[2] = 0;
        cout<<"Le going backwards, outside mat "<<confidenceCtr<<endl;
        return false;
    }
    else if(currentstate == ENTERTOPICKBALL) // only forward and timeout for now
    {
        if(stateTransition)
        {
            cout<<"Exiting mat "<<endl;
            stateTransition = false;
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        // Motion control : Default forward for this state
        controlSetpoint[0] = max_cols/2;
        controlSetpoint[1] = 0;
        controlSetpoint[2] = 0;
        cout<<"Le going forward, to pick ball "<<confidenceCtr<<endl;
        return false;
    }
    else if(currentstate == FINDDROPBIN)
    {
        if(stateTransition)
        {
            stateTransition = false;
            cout<<"Starting drop bin search"<<endl;
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        confidenceCtr = finddropbin(confidenceCtr,orig,maindisp,controlSetpoint);
        if(confidenceCtr == -1)
        {  // Motion control : Default forward for this state
            controlSetpoint[0] = max_cols/2;
            controlSetpoint[1] = 0;
            controlSetpoint[2] = 0;
            cout<<"Not detected drop bin"<<endl;
            confidenceCtr = 0;
        }
        else if(confidenceCtr > 10)
        {
            cout<<"Ball wala bin mil gya!! Utha lo :D"<<endl;
            return true;
        }
        else if(confidenceCtr!=0)
        {
            cout<<"Drop bin confidence: "<<confidenceCtr<<endl;
        }
        return false;
    }
    else if(currentstate == PICKUPBALL)		// TODO: needs review - how to do this
    {
        if(stateTransition)
        {
            stateTransition = false;
            cout<<"Starting first bin search"<<endl;
            confidenceCtr = 0;
            startTime = Clock::now();
        }
        confidenceCtr = firstbinsearch(confidenceCtr,orig,maindisp,controlSetpoint);
        if(confidenceCtr == -1)
        {  // Motion control : Default forward for this state
            controlSetpoint[0] = max_cols/2;
            controlSetpoint[1] = 0;
            controlSetpoint[2] = 0;
            cout<<"Not detected first bin"<<endl;
            confidenceCtr = 0;
        }
        else if(confidenceCtr > 10)
        {
            cout<<"First bin found."<<endl;
            return true;
        }
        else if(confidenceCtr!=0)
        {
            cout<<"First bin confidence: "<<confidenceCtr<<endl;
        }
        return false;
    }
    else if(currentstate == DONOTHING)
    {
        return false;
    }
    return false;
}

bool BinsTask::doTasks(Mat src, Vec3i &controlSetpoint, bool &timedOut)
{
    orig = src.clone();
    // make an array of states to follow
    // make an array of timers to follow with it

    if(stateComplete)
    {
        stateCounter++;
        if(stateCounter >= stateList.size())
            return true;
        currentstate = stateList[stateCounter];
        stateTransition = true;
        timelimit = stateTimeouts[stateCounter];
    }
    stateComplete = setState(orig,controlSetpoint,currentstate);
    rectangle(maindisp,Point(max_cols/2.0 - centerSlack,max_rows/2.0 - centerSlack),
              Point(max_cols/2.0 + centerSlack,max_rows/2.0 + centerSlack),Scalar(0,0,0),1,8,0);
    //imshow("State outputs display ", maindisp);
    //waitKey(5);
    if(findTimeDifference(Clock::now(),startTime)  > timelimit) // Le timeout
    {
        timedOut = true;
        stateComplete = true;
    }
    else
        timedOut = false;
    /*
    Timeout notes : Do not blindly reset the state machine due to timeout
    Some functions like exit mat and re enter mat works on timeout only
    if timeout is asserted, the calling function has to decide what to do after the timeout
    */
    return false;
}
