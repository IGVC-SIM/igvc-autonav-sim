#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // cout << "printthis";
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor(img,img,cv::COLOR_BGR2HSV);
    cv::Vec3i lb = {0,0,125}; //Lower limit for Lane's color
    cv::Vec3i up = {180,26,250};//Upper Limit for the Lane's Color
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size( 2*2 + 1, 2*2+1 ),cv::Point( 2, 2 ) );
    //element is a Kernel for eroding whatever tiny patches of white we detect in the image and also the skyline 
    //that gets detected due to it's similar color as the lane
    cv::inRange(img,lb,up,img);//Thresholding to generate a binary mask
    cv::Mat roi = img(cv::Rect(0,0,img.cols,200));//Region of interest i.e. the detected skyline
    cv::erode(roi,roi(cv::Rect(0,0,roi.cols,roi.rows)),element,cv::Point(-1,-1),4);//Eroding the skyline

    cv::imshow("view", img);//Viewing the final detection
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e) {
    cout << "Could not convert from '%s' to 'bgr8'." << msg->encoding.c_str();
  }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lanedetector");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    while(ros::ok()) {
    ros::Subscriber path_sub = nh.subscribe("/zed2i/zed_node/rgb/image_rect_color",1000,imageCallback);
    rate.sleep();
    ros::spinOnce();
    }
    cv::destroyAllWindows();   
}
