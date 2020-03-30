#include "Image_Thresholding_Tool/thresholding_tool.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "img_thresh");
    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera/image1", 1, thresholding_Tool::input);
  	//image_transport::Subscriber sub = it.subscribe("/front_camera/image_rect_color", 1, thresholding_Tool::input);
    QApplication a(argc, argv);
    thresholding_Tool w;
    w.show();

    return a.exec();
}
