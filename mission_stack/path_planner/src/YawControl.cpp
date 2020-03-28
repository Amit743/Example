#include "path_planner/YawControl.h"

YawControl::YawControl(int frameCount)
{
    canny_low = 10, canny_high = 100, thresholdVal = 50, minLineLength = 50, maxLineGap = 20;
    ind=-1;
    yawFramesRequired = frameCount;
}

YawControl::~YawControl(){}

double YawControl::findMedian(vector<double> values)
{
    size_t size = values.size();

    if (size == 0)
    {
        return 0;
    }
    else if (size == 1)
    {
        return values[0];
    }
    else
    {
        sort(values.begin(), values.end());
        if (size % 2 == 0)
        {
            return (values[size / 2 - 1] + values[size / 2]) / 2;
        }
        else
        {
            return values[size / 2];
        }
    }
}


vector<int> YawControl::filterOutliers(vector<Vec4i> lines, vector<double> angles)
{
    vector<int> filteredIndexes;

    double medianAngle = findMedian(angles);
    /*double med_dev;
    for(int i=0;i<angles.size();i++)
    {
        med_dev += pow(angles[i] - medianAngle,2);
    }
    med_dev = med_dev / angles.size();
    med_dev = sqrt(med_dev);*/
    for(int i=0;i<angles.size();i++)
    {
        // cout<<angles[i]<<" "<<med_dev<<endl;
        if(abs(angles[i] - medianAngle) < 0.5)
        {
            filteredIndexes.push_back(i);
        }
    }

    return filteredIndexes;
}

void YawControl::reset()
{
    yawVec.clear();
}

bool YawControl::findHeadingDiff(Mat orig)
{
    medianBlur( orig, src, 5);
    Canny(src, dst, canny_low, canny_high, 3);
    Mat src_gray,thresh;
    cvtColor(src,src_gray,CV_BGR2GRAY);
    threshold(src_gray,thresh,150,255,THRESH_BINARY);
    dst = dst - thresh;
    cvtColor(dst, cdst, CV_GRAY2BGR);

    vector<Vec4i> lines, filteredLinesV, filteredLinesH;
    vector<double> anglesV, anglesH;
    vector<int> filteredIndexesV, filteredIndexesH;

    HoughLinesP(dst, lines, 1, CV_PI/180, thresholdVal, minLineLength, maxLineGap);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        double angle = atan2 (abs(l[1]-l[3]),abs(l[0]-l[2])) * 180 / M_PI;
        // since abs is used, this angle can be used only for filtering and
        // not final computation.
        if(abs(90.0-angle) <= 45)
        {
            // line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 3, CV_AA);
            filteredLinesV.push_back(l);
            anglesV.push_back(angle);
        }
        if(abs(angle) < 45)
        {
            // line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 3, CV_AA);
            filteredLinesH.push_back(l);
            anglesH.push_back(angle);
        }
    }

    filteredIndexesH = filterOutliers(filteredLinesH, anglesH);
    // TODO: Temporary
    //filteredIndexesV = filterOutliers(filteredLinesV, anglesV);
    filteredIndexesV.clear();
    int angleCount = filteredIndexesH.size() + filteredIndexesV.size();

    if(angleCount==0)
        return false;

    headingDiff = 0;
    for(int i=0;i<filteredIndexesH.size();i++)
    {
        Vec4i l = filteredLinesH[filteredIndexesH[i]];
        double angle = atan2 ((l[1]-l[3]),(l[0]-l[2])) * 180 / M_PI;
        if(angle<45 && angle>-45)
        headingDiff += angle;
        else if (angle>135)
        headingDiff += (angle - 180);
        else
        headingDiff += (angle + 180);

        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }

    for(int i = 0; i < filteredIndexesV.size(); i++)
    {
        Vec4i l = filteredLinesV[filteredIndexesV[i]];
        double angle = atan2 ((l[1]-l[3]),(l[0]-l[2])) * 180 / M_PI;
        if(angle>=45 && angle<=135)
        headingDiff += (angle - 90);
        else
        headingDiff += (angle + 90);
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3, CV_AA);
    }
    headingDiff /= angleCount;

//    cout<<headingDiff<<endl;
    ind=(ind+1)%yawFramesRequired;
    if(yawVec.size()<yawFramesRequired)
        yawVec.push_back(headingDiff);
    else
        yawVec[ind] = headingDiff;

    if(yawVec.size()==yawFramesRequired)
    {
        nth_element(yawVec.begin(),yawVec.begin() + yawVec.size()/2,yawVec.end());
        headingDiff = yawVec[yawVec.size()/2];
        return true;
    }
    return false;
}
