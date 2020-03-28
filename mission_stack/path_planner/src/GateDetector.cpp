#include "path_planner/GateDetector.h"


GateDetector::GateDetector()
{
    sobelThresholdX = 35;
    sobelThresholdY = 35;
    morph_size = 1;
    nCenters = 30;
    nCentersIndex = -1;
    scale = 1;
    delta = 0;
    ddepth = CV_16S;
};

GateDetector::~GateDetector(){}

bool GateDetector::detectGate(Mat orig)
{
    /*Mat rotmat = getRotationMatrix2D(Point2f(max_cols/2,max_rows/2), 10.0, 1);
    warpAffine(orig.clone(), orig, rotmat, Size(max_cols,max_rows));
    imshow("Rotated",orig);
    waitKey(1);*/

    Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ));

	GaussianBlur( orig, src, Size(3,3), 0, 0, BORDER_DEFAULT );

	cvtColor( src, src_gray, CV_BGR2GRAY );

    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );
    // imshow("Sobel",abs_grad_x);
    dilate(abs_grad_x.clone(),abs_grad_x,element);
    threshold(abs_grad_x,abs_grad_x,sobelThresholdX,255, THRESH_BINARY);

    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );

    convertScaleAbs( grad_y, abs_grad_y );
    
    //dilate(abs_grad_y.clone(),abs_grad_y,element);
    threshold(abs_grad_y,abs_grad_y,sobelThresholdY,255, THRESH_BINARY);

    Mat tempThresh;
    threshold(src_gray,tempThresh,140,255,THRESH_BINARY);
    //imshow("tempThresh",tempThresh);

    Mat abs_grad_x_temp = abs_grad_x - abs_grad_y;
    abs_grad_x_temp = abs_grad_x - tempThresh;
    Mat abs_grad_y_temp = abs_grad_y - abs_grad_x;
    abs_grad_y_temp = abs_grad_y - tempThresh;
    abs_grad_x = abs_grad_x_temp;
	abs_grad_y = abs_grad_y_temp;
    // imshow("abs_grad_x",abs_grad_x);
    // imshow("abs_grad_y",abs_grad_y);
    // waitKey(1);

    cdst = orig.clone();

    vector<Vec4i> lines,filteredLinesH,filteredLinesV;
	HoughLinesP(abs_grad_x, lines, 1, CV_PI/180, 50, 50, 10 );

	for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        double angle = atan2 (abs(l[1]-l[3]),abs(l[0]-l[2])) * 180 / M_PI;
        if(abs(90.0-angle) < 10)
        {
            filteredLinesV.push_back(l);
        }
    }

    Mat blankMat = Mat::zeros(orig.size(), CV_8UC3);
    for(int i=0;i<filteredLinesV.size();i++)
    {
            Vec4i l = filteredLinesV[i];
            line( blankMat, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 5, CV_AA);
    }

    vector<vector<Point> > contours;
    cvtColor(blankMat.clone(),blankMat,CV_BGR2GRAY);
    findContours( blankMat, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );

    Mat topBarCheckCrop;
	vector<Rect> filteredSquares;
    for( int i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        int height = boundRect[i].height;
        Point tl = Point(boundRect[i].tl().x-0.05*height,boundRect[i].tl().y-0.05*height);
        Point br = Point(boundRect[i].tl().x+1.05*height,boundRect[i].tl().y+0.1*height);

        rectangle( cdst, boundRect[i], Scalar(0,0,255), 2, 8, 0 );
        rectangle( cdst, tl,br, Scalar(0,0,255), 2, 8, 0 );

        Rect topBarCheckImgRect = Rect(tl.x,tl.y,br.x-tl.x+1,br.y-tl.y+1) & Rect(0, 0, src.cols, src.rows);
		if(topBarCheckImgRect.width <= 0 || topBarCheckImgRect.height <= 0 )
			continue;

		// New check for top bar
        topBarCheckCrop = abs_grad_y(topBarCheckImgRect);
        filteredLinesH.clear();
        lines.clear();

        HoughLinesP(topBarCheckCrop, lines, 1, CV_PI/180, 50, 50, 10 );
		for(int i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            double angle = atan2 (abs(l[1]-l[3]),abs(l[0]-l[2])) * 180 / M_PI;
            if(abs(angle) < 5)
            {
                filteredLinesH.push_back(l);
            }
        }

		if(filteredLinesH.size() == 0)
			continue;

		int max_line_index = -1;
		int max_line_size = 0;
		for(unsigned int i= 0;i<filteredLinesH.size();i++)
		{
			if(max_line_size < (filteredLinesH[i][2]-filteredLinesH[i][0]) )
			{
				max_line_index = i;
				max_line_size  = (filteredLinesH[i][2]-filteredLinesH[i][0]);
			}
		}
		double barRatio = max_line_size/((float)height);

		if(barRatio < 1.1 && barRatio > 0.7) // le tuned somewhat, still has some false positives
		{
			Rect foundRect = Rect(tl.x,tl.y,(1.15*height),(1.1*height)) & Rect(0,0,src.cols,src.rows);
			filteredSquares.push_back(foundRect);
			rectangle(cdst,foundRect,Scalar(0,0,0),2,8,0);
		}
    }

    if(filteredSquares.size()==0)
    {
        return false;
    }

    // If there is nothing to compare to and more than 1 possible gate is detected
    if(lastCenters.size() == 0 && filteredSquares.size()!=1)
        return false;

    if(lastCenters.size() == 0)
    {
        Rect r = filteredSquares[0];
        lastCenters.push_back(Point(r.x+r.width/2,r.y+r.height/2));
    }

    int index = 0;
    double minDev = DBL_MAX;

    for(int i=0;i<filteredSquares.size();i++)
    {
        double dev = 0;
        Rect r = filteredSquares[0];
        Point p1 = Point(r.x+r.width/2,r.y+r.height/2);
        for(int j=0;j<lastCenters.size();j++)
        {
            Point p2 = lastCenters[j];
            dev+= dist(p1,p2);
        }
        if(dev<minDev)
        {
            minDev = dev;
            index = i;
        }
    }

    gateRect = filteredSquares[index];
    gateCenter = Point(gateRect.x+gateRect.width/2,gateRect.y+gateRect.height/2);

    nCentersIndex = (nCentersIndex+1)%nCenters;

    if(lastCenters.size()<nCenters)
    {
        lastCenters.push_back(gateCenter);
        return false;
    }
    else
    {
        lastCenters[nCentersIndex] = gateCenter;
        int count = 0;
        for(int i=0;i<lastCenters.size();i++)
        {
            if(dist(gateCenter,lastCenters[i])<50)
                count++;
        }
        if(count<0.6*nCenters)
        {
            return false;
        }
        line(cdst,Point(src.cols/2,0),Point(src.cols/2,src.rows),Scalar(0,0,255),2,2,0);
        rectangle( cdst, gateRect, Scalar(255,0,0), 2, 8, 0 );
        circle(cdst,gateCenter,5,Scalar(255,00,0),2,5,0);
        return true;
    }
}

double GateDetector::dist(Point p1,Point p2)
{
    return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}
