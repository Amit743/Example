#ifndef THRESHOLDING_TOOL_H
#define THRESHOLDING_TOOL_H

#include "ui_thresholding_tool.h"
#include <QMainWindow>
#include <QPixmap>
#include <QTimer>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

static Mat frame;

enum Color_spaces { HSV, RGB, BGR, GRAY} ;

namespace Ui {
class thresholding_Tool;
}

class thresholding_Tool : public QMainWindow
{
    Q_OBJECT

public:
    explicit thresholding_Tool(QWidget *parent = 0);
    ~thresholding_Tool();
    static void input(const sensor_msgs::ImageConstPtr&);
    Ui::thresholding_Tool *ui;

public slots:
    void loop();
    void play();
    void save();
    void open_config(int);
    void add_process_sequence(int);
    void h_1(int);
    void s_1(int);
    void v_1(int);
    void h_2(int);
    void s_2(int);
    void v_2(int);
    void H_1(int);
    void S_1(int);
    void V_1(int);
    void H_2(int);
    void S_2(int);
    void V_2(int);
    void E_kernel(int);
    void D_kernel(int);
    void sob(int);
    void opn(int);
    void Threshold(int);
    void Rwidth(int);
    void Rheight(int);

private:
    cv::Mat src, Hsv, hsv1, hsv2, output, erosion_output, dilation_output, detected_edges, contour_img;

    void HSV_1();
    void HSV_2();
    void merge();
    void erosion();
    void dilation();
    void open_morph();
    void sobel();
    void houghLinesP();
    void contour();
    void output_feed(cv::Mat);

    int hsv[12];
    Color_spaces output_color_space=RGB;
    int sobel_kernel=1;
    int open_morph_kernel=1;
    int object_no=0;
    int threshold=0;
    int width=10;
    int height=10;
    int ratio=2;
    int kernel_size=3;
    int erosion_size=1;
    int dilation_size=1;
    int minLineGap=0;
    int minLineLength=0;
    QTimer *timer;
    cv::VideoCapture cap;
    std::string CONFIG_PATH;

};

#endif // THRESHOLDING_TOOL_H
