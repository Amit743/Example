#include "path_planner/AlignBin.h"

AlignBin::AlignBin()
{
    bin_found_confidence = 0;
}

AlignBin::~AlignBin()
{

}

bool AlignBin::findBinToChoose(int lBins, Mat labelsBins, Mat statsBins, Rect cropRect, int &max_area_index)
{
    #ifdef showImages
    imshow("crop",thresh(cropRect));
    #endif
    max_area_index = -1;
    int max_area= 0 ;
    // Find connectedcomponent with largest bounding box
    for(int i = 1; i <lBins; i++)
    {
        double bin_area = statsBins.at<int>(i,CC_STAT_AREA);
        double bin_width = statsBins.at<int>(i,CC_STAT_WIDTH);
        double bin_height = statsBins.at<int>(i,CC_STAT_HEIGHT);

        Rect tempRect = Rect(cropRect.x+statsBins.at<int>(i,CC_STAT_LEFT),
                            cropRect.y+statsBins.at<int>(i,CC_STAT_TOP),
                            bin_width, bin_height);
        rectangle(draw,tempRect,Scalar(255,0,0),2,8,0);

        if(bin_area < 35)
            continue;

        if(!bin_found && bin_area/(bin_outer_crop.width*bin_outer_crop.height) > 0.05)
            continue;

        if(bin_area/(bin_width*bin_height) < 0.5) // less than 50% filled
            continue;

        if(bin_width >  BIN_REQUIRED_RATIO * bin_height)
            continue;

        if(bin_width * 1.5 < bin_height)
            continue;

        if( bin_area > max_area)
        {
            max_area = bin_area;
            max_area_index = i;
        }
    }
    if(max_area_index == -1)
        return false;

    binCenter.x = cropRect.x + (statsBins.at<int>(max_area_index,CC_STAT_LEFT)
                    + statsBins.at<int>(max_area_index,CC_STAT_WIDTH)/2);
    binCenter.y = cropRect.y + (statsBins.at<int>(max_area_index,CC_STAT_TOP)
                    + statsBins.at<int>(max_area_index,CC_STAT_HEIGHT)/2);
    return true;
}

void AlignBin::updateInnerCrop(Mat statsBins, int index)
{
    bin_inner_crop.x = binCenter.x - BIN_INNER_CROP_RATIO*statsBins.at<int>(index,CC_STAT_WIDTH);
    bin_inner_crop.y = binCenter.y - BIN_INNER_CROP_RATIO*statsBins.at<int>(index,CC_STAT_HEIGHT);
    bin_inner_crop.width = 2*BIN_INNER_CROP_RATIO*statsBins.at<int>(index,CC_STAT_WIDTH);
    bin_inner_crop.height = 2*BIN_INNER_CROP_RATIO*statsBins.at<int>(index,CC_STAT_HEIGHT);
    bin_inner_crop = bin_inner_crop & Rect(0,0,max_cols,max_rows);
}


void AlignBin::initializeInnerCrop(Mat statsBins, int index)
{
    bin_inner_crop.x = bin_outer_crop.x
                     + statsBins.at<int>(index,CC_STAT_LEFT)
                     + statsBins.at<int>(index,CC_STAT_WIDTH)/2
                     - BIN_INNER_CROP_RATIO*statsBins.at<int>(index,CC_STAT_WIDTH);
    bin_inner_crop.y = bin_outer_crop.y
                     + statsBins.at<int>(index,CC_STAT_TOP)
                     + statsBins.at<int>(index,CC_STAT_HEIGHT)/2
                     - BIN_INNER_CROP_RATIO*statsBins.at<int>(index,CC_STAT_HEIGHT);
    bin_inner_crop.width  = 2*BIN_INNER_CROP_RATIO*statsBins.at<int>(index,CC_STAT_WIDTH);
    bin_inner_crop.height = 2*BIN_INNER_CROP_RATIO*statsBins.at<int>(index,CC_STAT_HEIGHT);

    bin_inner_crop = bin_inner_crop & Rect(0,0,max_cols,max_rows);
}

bool AlignBin::findBins(Mat orig, bool initial)
{
    //cout<<"BF:"<<bin_found<<" LBF:"<<last_bin_found<<" NF:"<<bin_not_found_ctr<<endl;
    //cerr<<"In find bins"<<endl;
    Mat frame;
    draw = orig.clone();

    cvtColor(orig, frame, CV_BGR2HSV);

    Mat thresh1, thresh2, thresh3;

    // inRange(frame,Scalar(82,0,0),Scalar(111,255,255),thresh1); // Blue pool-bot
    // inRange(frame,Scalar(0,100,0), Scalar(11,255,255),thresh2); // Red-1 pool-bot


    inRange(frame,Scalar(100,150,0),Scalar(140,255,255),thresh1); // Blue simulator
    inRange(frame,Scalar(0,100,100), Scalar(10,255,255),thresh2); // Red-1 simulator

    // int valHighTemp = VH2;
    // while(true)
    // {
    //     if(valHighTemp < 30)
    //         break;

    // inRange(frame,Scalar(160,100,0), Scalar(180,255,255), thresh3); //Red-2 // pool-bot

        inRange(frame,Scalar(160,100,100), Scalar(180,255,255), thresh3); // Red-2 simulator
    //     double nonZero = countNonZero(thresh3);

    //     if(nonZero/(max_rows*max_cols) > 0.05)
    //         valHighTemp -= 5;
    //     else
    //         break;
    // }

    bitwise_or(thresh1,thresh2,thresh);
    bitwise_or(thresh3,thresh,thresh);

//    if(!bin_found)
//        thresh = thresh3;
#ifdef showImages
imshow("Thresh",thresh);
#endif

    Mat binsCrop;

    Mat element = getStructuringElement( MORPH_RECT, Size( 5, 5 ));

    dilate(thresh(bin_outer_crop),thresh(bin_outer_crop),element);

    // imshow("Thresh",thresh);
    Rect cropRect;

    if(bin_found)
    {
        cropRect = bin_inner_crop;
    }
    else
    {
        cropRect = bin_outer_crop;
    }
    binsCrop = thresh(cropRect);

    rectangle(draw,bin_outer_crop,Scalar(0,255,0),2,8,0);

    Mat labelsBins,statsBins,centroidBins;
    int lBins = connectedComponentsWithStats(binsCrop, labelsBins, statsBins, centroidBins);

    if(lBins != 1)
    {
        int index;
        //cerr<<"finding bin to choose"<<endl;

        last_bin_found = findBinToChoose(lBins, labelsBins, statsBins, cropRect, index);
        //cerr<<"found"<<endl;

        if(bin_found) // inner crop has been initialized before
        {
            if(last_bin_found)
            {
                bin_not_found_ctr = 0;
                circle(draw,binCenter,2,Scalar(0,0,255),2,8,0);
                //cerr<<"update inner"<<endl;

                updateInnerCrop(statsBins,index);
                //cerr<<"updated"<<endl;

                rectangle(draw,bin_inner_crop,Scalar(0,255,0),2,8,0);

                if(initial)
                {
                    bin_found_confidence++;
                    if(bin_found_confidence > 5)
                    {
                        bin_found_confidence = 0;
                        cout<<"Bin found"<<endl;
                        return true;
                    }
                }
                else
                {
                    double bin_area = statsBins.at<int>(index,CC_STAT_AREA);

                    if(bin_area/(bin_outer_crop.width*bin_outer_crop.height) > BIN_AREA_THRESHOLD)
                    {
                        bin_area_exceeded_ctr ++;
                        if(bin_area_exceeded_ctr > 10)
                        {
                            cout<<"Bin area exceeded"<<endl;
                            return true;
                        }
                    }
                }
            }
            else
            {
                bin_not_found_ctr++;
                if(bin_not_found_ctr > 10)
                {
                    last_bin_found = false;
                    bin_found = false;
                }

                bin_found_confidence = 0;
            }
        } // reset inner crop
        else
        {
            //cerr<<"initialize ctop"<<endl;

            initializeInnerCrop(statsBins,index);
            //cerr<<"initialized"<<endl;

            rectangle(draw,bin_inner_crop,Scalar(0,0,255),2,8,0);
            if(last_bin_found)
            {
                bin_not_found_ctr = 0;
                bin_found = true;
            }
            else
            {
                bin_found = false;
            }
        }
    }
    else
    {
        bin_found = false;
        last_bin_found = false;
    }
    #ifdef showImages
    imshow("Draw",draw);
    waitKey(1);
    #endif
    return false;
}
