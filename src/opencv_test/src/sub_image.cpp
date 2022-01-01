//
// Created by chen on 2021/11/20.
//

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr= cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img=ptr->image.clone();

    cv::cuda::GpuMat d_img(img);
    cv::cuda::bitwise_not(d_img,d_img);
    d_img.download(img);

    cv::imshow("image",img);
    cv::waitKey(1);
}

int main(int argc,char **argv)
{
    cv::Mat img(100,100,CV_8UC1);
    cv::cuda::GpuMat d_img(img);
    cv::cuda::bitwise_not(d_img,d_img);
    d_img.download(img);

    ros::init(argc, argv, "multi_image_node");

    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Subscriber sub_img0 = n.subscribe("/camera/left/image_raw", 100, img0_callback);

    cout<<"wait image"<<endl;

    ros::spin();

    return 0;
}