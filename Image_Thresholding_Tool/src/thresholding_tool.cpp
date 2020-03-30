#include "Image_Thresholding_Tool/thresholding_tool.h"
#include "ui_thresholding_tool.h"

thresholding_Tool::thresholding_Tool(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::thresholding_Tool)
{
	ui->setupUi(this);
	timer = new QTimer(this);

	connect(timer, SIGNAL(timeout()),this,SLOT(loop()));
	connect(ui->Play, SIGNAL(pressed()), this, SLOT(play()));
	connect(ui->save, SIGNAL(pressed()), this, SLOT(save()));
  connect(ui->object_list,SIGNAL(currentIndexChanged(int)),this,SLOT(open_config(int)));
        connect(ui->process_list,SIGNAL(currentIndexChanged(int)),this,SLOT(add_process_sequence(int)));
        connect(ui->hmin_1,SIGNAL(valueChanged(int)),this,SLOT(h_1(int)));
        connect(ui->smin_1,SIGNAL(valueChanged(int)),this,SLOT(s_1(int)));
        connect(ui->vmin_1,SIGNAL(valueChanged(int)),this,SLOT(v_1(int)));
        connect(ui->hmax_1,SIGNAL(valueChanged(int)),this,SLOT(H_1(int)));
        connect(ui->smax_1,SIGNAL(valueChanged(int)),this,SLOT(S_1(int)));
        connect(ui->vmax_1,SIGNAL(valueChanged(int)),this,SLOT(V_1(int)));
	      connect(ui->hmin_2,SIGNAL(valueChanged(int)),this,SLOT(h_2(int)));
        connect(ui->smin_2,SIGNAL(valueChanged(int)),this,SLOT(s_2(int)));
        connect(ui->vmin_2,SIGNAL(valueChanged(int)),this,SLOT(v_2(int)));
        connect(ui->hmax_2,SIGNAL(valueChanged(int)),this,SLOT(H_2(int)));
        connect(ui->smax_2,SIGNAL(valueChanged(int)),this,SLOT(S_2(int)));
        connect(ui->vmax_2,SIGNAL(valueChanged(int)),this,SLOT(V_2(int)));
        connect(ui->erode_kernel,SIGNAL(valueChanged(int)),this,SLOT(E_kernel(int)));
        connect(ui->dilate_kernel,SIGNAL(valueChanged(int)),this,SLOT(D_kernel(int)));
        connect(ui->sobel_kernel,SIGNAL(valueChanged(int)),this,SLOT(sob(int)));
        connect(ui->open_morph,SIGNAL(valueChanged(int)),this,SLOT(opn(int)));
        connect(ui->canny_threshold,SIGNAL(valueChanged(int)),this,SLOT(Threshold(int)));
        connect(ui->width,SIGNAL(valueChanged(int)),this,SLOT(Rwidth(int)));
        connect(ui->height,SIGNAL(valueChanged(int)),this,SLOT(Rheight(int)));
}

thresholding_Tool::~thresholding_Tool()
{
    delete ui;
}

void thresholding_Tool::input(const sensor_msgs::ImageConstPtr& msg)
{
      cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
    }
	frame= cv_ptr->image;
}

void thresholding_Tool::save()
{
	std::cout<<"Parameters saved\n";
        ifstream f(CONFIG_PATH);
        if(f.eof())
	{
		std::cout<<"Config File missing\n";
                ofstream ofs(CONFIG_PATH);
                ofs.close();
	}
        else
	{
		ofstream f(CONFIG_PATH,ios::ate);
		f << "HSV 1 THRESHOLDING PARAMETERS:\nLow_H = " << hsv[0];
		f << "\nLow_S = " << hsv[1];
		f << "\nLow_V = " << hsv[2];
		f << "\nHigh_H = " << hsv[3];
		f << "\nHigh_S = " << hsv[4];
		f << "\nHigh_V = " << hsv[5];
    f << "\nHSV 2 THRESHOLDING PARAMETERS:\nLow_H = " << hsv[6];
		f << "\nLow_S = " << hsv[7];
		f << "\nLow_V = " << hsv[8];
		f << "\nHigh_H = " << hsv[9];
		f << "\nHigh_S = " << hsv[10];
		f << "\nHigh_V = " << hsv[11];
    f << "\nEROSION:\nKernel_size = " << erosion_size;
    f << "\nDILATION:\nKernel_size = " << dilation_size;
		f << "\nSOBEL PARAMETERS:\nKernel_size = " << sobel_kernel;
		f << "\nMORPH PARAMETERS:\nKernel_size = " << open_morph_kernel;
    f << "\nMin_line_gap = " << minLineGap;
    f << "\nMin_line_length = " << minLineLength;
		f << "\nCANNY THRESHOLD PARAMETERS:\nThreshold = " << threshold;
		f << "\nRatio = " << ratio;
		f << "\nKernel_size = " << kernel_size;
		f << "\nCONTOUR RECTANGLE PARAMETERS:\nHeight = " << height;
		f << "\nWidth = " << width;
		f.close();
	}
}

void thresholding_Tool::open_config(int index)
{
  object_no = index;
  switch(index)
  {
    case 0 : 	CONFIG_PATH = ros::package::getPath("Image_Thresholding_Tool") + "/config/gate_config.txt";
              break;
    case 1 : 	CONFIG_PATH = ros::package::getPath("Image_Thresholding_Tool") + "/config/bluebucket_config.txt";
              break;
    case 2 : 	CONFIG_PATH = ros::package::getPath("Image_Thresholding_Tool") + "/config/redbucket_config.txt";
              break;
    case 3 : 	CONFIG_PATH = ros::package::getPath("Image_Thresholding_Tool") + "/config/redflare_config.txt";
              break;
    case 4 : 	CONFIG_PATH = ros::package::getPath("Image_Thresholding_Tool") + "/config/yellowflare_config.txt";
              break;
  }
}

void thresholding_Tool::loop()
{
		if(!frame.empty()){
		cv::cvtColor(frame, src, CV_BGR2RGB);
		ui->input_feed->setPixmap(QPixmap::fromImage(QImage(src.data, src.cols, src.rows,src.step, QImage::Format_RGB888)));
          } 
}

void thresholding_Tool::play()
{
	if(timer->isActive())
	{
		timer->stop();
		ui->Play->setText("play");
	}
	else
	{
		timer->start(50);
		ui->Play->setText("pause");
	}
}

void thresholding_Tool::add_process_sequence(int process_no)
{
  switch(process_no)
  {
    case 0 : ui->process_sequence->setText("HSV Thresholding 1\n");
             HSV_1();
             break;
    case 1 : ui->process_sequence->setText("HSV Thresholding 2\n");
             HSV_2();
             break;
    case 2 : ui->process_sequence->setText("Merging HSV1 and HSV2\n");
             merge();
             break;
    case 3 : ui->process_sequence->setText("Erosion\n");
             erosion();
             break;
    case 4 : ui->process_sequence->setText("Dilation\n");
             dilation();
             break;
    case 5 : ui->process_sequence->setText("Open Morphing\n");
             open_morph();
             break;
    case 6 : ui->process_sequence->setText("Sobel Edge Detection\n");
             sobel();
             break;
    case 7 : ui->process_sequence->setText("Hough Lines Probablistic\n");
             houghLinesP();
             break;
    case 8 : ui->process_sequence->setText("Contour\n");
             contour();
             break;
  }
}

void thresholding_Tool::HSV_1()
{
  hsv1=frame.clone();
  cv::Scalar maxHSV , minHSV;
  cv::Mat hsv1_mask1, hsv1_mask2;
  cv::cvtColor(hsv1, hsv1, cv::COLOR_BGR2HSV);
  if(hsv[0]>0)
  {
    minHSV = cv::Scalar(hsv[0],hsv[1],hsv[2]);
    maxHSV = cv::Scalar(hsv[3],hsv[4],hsv[5]);
    cv::inRange(hsv1, minHSV, maxHSV, hsv1_mask1);
  }
  else
  {
    minHSV = cv::Scalar(0,hsv[1],hsv[2]);
          maxHSV = cv::Scalar(hsv[3],hsv[4],hsv[5]);
          cv::inRange(hsv1, minHSV, maxHSV, hsv1_mask1);
  	minHSV = cv::Scalar(180+hsv[0],hsv[1],hsv[2]);
          maxHSV = cv::Scalar(180,hsv[4],hsv[5]);
          cv::inRange(hsv1, minHSV, maxHSV, hsv1_mask2);
  	hsv1=hsv1_mask1|hsv1_mask2;
    output_color_space = GRAY;
    output_feed(hsv1);
  }
}

  void thresholding_Tool::HSV_2()
  {
    hsv2=frame.clone();
    cv::Scalar maxHSV , minHSV;
    cv::Mat hsv2_mask1, hsv2_mask2;
    cv::cvtColor(hsv2, hsv2, cv::COLOR_BGR2HSV);
    if(hsv[6]>0)
    {
      minHSV = cv::Scalar(hsv[6],hsv[7],hsv[8]);
      maxHSV = cv::Scalar(hsv[9],hsv[10],hsv[11]);
      cv::inRange(hsv2, minHSV, maxHSV, hsv2_mask1);
    }
    else
    {
      minHSV = cv::Scalar(0,hsv[7],hsv[8]);
      maxHSV = cv::Scalar(hsv[9],hsv[10],hsv[11]);
      cv::inRange(hsv2, minHSV, maxHSV, hsv2_mask1);
    	minHSV = cv::Scalar(180+hsv[6],hsv[7],hsv[8]);
      maxHSV = cv::Scalar(180,hsv[10],hsv[11]);
      cv::inRange(hsv2, minHSV, maxHSV, hsv2_mask2);
    	hsv2=hsv2_mask1|hsv2_mask2;
      output_color_space = GRAY;
      output_feed(hsv2);
    }
  }

  void thresholding_Tool::merge()
  {
    Hsv = hsv1|hsv2;
    output_color_space = GRAY;
    output_feed(Hsv);
  }

  void thresholding_Tool::erosion()
  {
    switch(output_color_space)
    {
      case HSV : break;
      case RGB : cv::cvtColor( output, output, cv::COLOR_RGB2HSV);
                 break;
      case BGR : cv::cvtColor( output, output, cv::COLOR_BGR2HSV);
                 break;
      case GRAY : //cv::cvtColor( output, output, cv::COLOR_GRAY2HSV);
                  break;
    }

    output_color_space = HSV;

    cv::Mat element = getStructuringElement( MORPH_RECT,
                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                         Point( erosion_size, erosion_size ) );
    erode( output, erosion_output, element );
    output_feed(erosion_output);
  }

  void thresholding_Tool::dilation()
  {
    switch(output_color_space)
    {
      case HSV : break;
      case RGB : cv::cvtColor( output, output, cv::COLOR_RGB2HSV);
                 break;
      case BGR : cv::cvtColor( output, output, cv::COLOR_BGR2HSV);
                 break;
      case GRAY : //cv::cvtColor( output, output, cv::COLOR_GRAY2HSV);
                  break;
    }

    output_color_space = HSV;

    cv::Mat element = getStructuringElement( MORPH_RECT,
                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                         Point( dilation_size, dilation_size ) );
    erode( output, dilation_output, element );
    output_feed(dilation_output);
  }

  void thresholding_Tool::open_morph()
  {
    Mat hsv_channels[3];
    switch(output_color_space)
    {
      case HSV : split( output, hsv_channels);
		 output = hsv_channels[2];
                 break;
      case RGB : cv::cvtColor( output, output, cv::COLOR_RGB2HSV);
                 break;
      case BGR : cv::cvtColor( output, output, cv::COLOR_BGR2HSV);
                 break;
      case GRAY : break;
    }
    output_color_space = GRAY;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,cv::Size( open_morph_kernel,open_morph_kernel ),cv::Point(open_morph_kernel/2,open_morph_kernel/2) );
    cv::morphologyEx( output, output, cv::MORPH_OPEN, element);
    output_feed(output);
  }

  void thresholding_Tool::sobel()
  {
    cv::Mat gray, hsv_channels[3], gradient_x, gradient_y, absolute_gradient_x, absolute_gradient_y;
    switch(output_color_space)
    {
      case HSV : split( output, hsv_channels);
		 gray = hsv_channels[2];
                 break;
      case RGB : cv::cvtColor( output, gray, cv::COLOR_RGB2HSV);
                 break;
      case BGR : cv::cvtColor( output, gray, cv::COLOR_BGR2HSV);
                 break;
      case GRAY : break;
    }
    output_color_space = GRAY;
    Sobel( gray, gradient_x, CV_16S, 1, 0, sobel_kernel);
    Sobel( gray, gradient_y, CV_16S, 0, 1, sobel_kernel);
    convertScaleAbs( gradient_x, absolute_gradient_x);
    convertScaleAbs( gradient_y, absolute_gradient_y);
    addWeighted( absolute_gradient_x, 0.5, absolute_gradient_y, 0.5, 0, output);
    output_feed(output);
  }

  void thresholding_Tool::houghLinesP()
  {
    cv::Mat hsv_channels[3], HoughLinesP, line_img;
    std::vector <cv::Vec4i> lines;
    switch(output_color_space)
    {
      case HSV : split( output, hsv_channels);
		 HoughLinesP = hsv_channels[2];
                 break;
      case RGB : cv::cvtColor( output, HoughLinesP, cv::COLOR_RGB2GRAY);
                 break;
      case BGR : cv::cvtColor( output, HoughLinesP, cv::COLOR_BGR2GRAY);
                 break;
      case GRAY : HoughLinesP = output.clone();
                  break;
    }
    output_color_space = GRAY;
    cv::HoughLinesP( HoughLinesP, lines, 1, CV_PI/180, threshold, minLineLength, minLineGap);
    for( size_t i = 0; i < lines.size(); i++)
    {
      cv::Vec4i line_end_co_ords = lines[i];
      int x1 = line_end_co_ords[0], x2 = line_end_co_ords[2], y1 = line_end_co_ords[1], y2 = line_end_co_ords[3];
      double angle = (x1==x2)?90:atan(1.0*(y2-y1)/(x2-x1))*180/CV_PI;
      if(abs(angle)>83)
      {
        line( line_img, cv::Point( x1, y1), cv::Point( x2, y2), cv::Scalar(0,255,0), 3, cv::LINE_AA);
      }
    }
    cv::cvtColor( line_img, output, cv::COLOR_BGR2RGB);
    output_feed(output);
  }

  void thresholding_Tool::contour()
  {
    cv::Mat gray, hsv_channels[3];
    switch(output_color_space)
    {
      case HSV : split( output, hsv_channels);
		 gray = hsv_channels[2];
                 break;
      case RGB : cv::cvtColor( output, gray, cv::COLOR_RGB2GRAY);
                 break;
      case BGR : cv::cvtColor( output, gray, cv::COLOR_BGR2GRAY);
                 break;
      case GRAY : gray = output.clone();
                  break;
    }
    output_color_space = GRAY;
    Canny( gray, detected_edges, threshold, threshold*ratio, kernel_size);
    std::vector<vector<Point> > Contour;
    std::vector<Vec4i> hierarchy;
    findContours( detected_edges, Contour, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
    std::vector<vector<Point> > contour_poly(Contour.size());
    std::vector<Rect> boundRect(Contour.size());
    std::vector<Point2f> centers(Contour.size());
    for(size_t i = 0; i<Contour.size(); i++)
    {
      approxPolyDP(Contour[i],contour_poly[i],3,true);
      boundRect[i] = boundingRect(contour_poly[i]);
    }
    cv::Mat drawing = Mat::zeros(detected_edges.size(),CV_8UC3);
    for(size_t i = 0;i<Contour.size();i++)
    {
      cv::drawContours( drawing, contour_poly, (int)i, (0,0,255));
      if(boundRect[i].width>width && boundRect[i].height>height)
      rectangle( contour_img, boundRect[i].tl(), boundRect[i].br(), (0,0,255), 2);
    }
    output_color_space = RGB;
    output_feed(contour_img);
  }

  void thresholding_Tool::output_feed(cv::Mat output_img)
  {
    switch(output_color_space)
   {
   	case HSV : cv::cvtColor(output_img, output, cv::COLOR_HSV2RGB);
                   break;
	case RGB : output = output_img.clone();
		   break;
	case BGR : cv::cvtColor(output_img, output, cv::COLOR_BGR2RGB);
                   break;
	case GRAY : cv::cvtColor(output_img, output, cv::COLOR_GRAY2RGB);
                   break;
   }
    output_color_space = RGB;
    ui->processed_feed->setPixmap(QPixmap::fromImage(QImage( output.data, output.cols, output.rows, output.step, QImage::Format_RGB888)));
  }

void thresholding_Tool::h_1(int value)
{
	hsv[0]=value;
}
void thresholding_Tool::s_1(int value)
{
	hsv[1]=value;
}
void thresholding_Tool::v_1(int value)
{
	hsv[2]=value;
}
void thresholding_Tool::H_1(int value)
{
	hsv[3]=value;
}
void thresholding_Tool::S_1(int value)
{
	hsv[4]=value;
}
void thresholding_Tool::V_1(int value)
{
	hsv[5]=value;
}
void thresholding_Tool::h_2(int value)
{
	hsv[6]=value;
}
void thresholding_Tool::s_2(int value)
{
	hsv[7]=value;
}
void thresholding_Tool::v_2(int value)
{
	hsv[8]=value;
}
void thresholding_Tool::H_2(int value)
{
	hsv[9]=value;
}
void thresholding_Tool::S_2(int value)
{
	hsv[10]=value;
}
void thresholding_Tool::V_2(int value)
{
	hsv[11]=value;
}
void thresholding_Tool::opn(int value)
{
	open_morph_kernel=value;
}
void thresholding_Tool::sob(int value)
{
	sobel_kernel=value;
}
void thresholding_Tool::Threshold(int value)
{
	threshold=value;
}
void thresholding_Tool::Rwidth(int value)
{
	width=value;
}
void thresholding_Tool::Rheight(int value)
{
	height=value;
}
void thresholding_Tool::E_kernel(int value)
{
  erosion_size=value;
}
void thresholding_Tool::D_kernel(int value)
{
  dilation_size=value;
}
