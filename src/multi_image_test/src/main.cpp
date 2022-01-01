//
// Created by chen on 2021/9/20.
//
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;


const int QUEUE_SIZE=20;
const double DELAY=0.005;

/*
bool cmp_func(pair<double,cv::Mat>& a,pair<double,cv::Mat>& b){
    return b.first<a.first;
}
typedef std::priority_queue<pair<double,cv::Mat>,vector<pair<double,cv::Mat>>, decltype(cmp_func)*> TimeImageQueue;
TimeImageQueue img_buf;
std::mutex m_buf;
 */

queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> seg0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<sensor_msgs::ImageConstPtr> seg1_buf;
std::mutex m_buf;

size_t cnt=0;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    if(img0_buf.size()<QUEUE_SIZE){
        img0_buf.push(img_msg);
    }
    m_buf.unlock();
}

void seg0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    if(seg0_buf.size()<QUEUE_SIZE)
        seg0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cnt++;
    if(cnt==40){
        return;
    }

    m_buf.lock();
    if(img1_buf.size()<QUEUE_SIZE)
        img1_buf.push(img_msg);
    m_buf.unlock();
}

void seg1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    if(seg1_buf.size()<QUEUE_SIZE)
        seg1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr= cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    return ptr->image.clone();
}


struct SegImage{
    cv::Mat img0,seg0,img1,seg1;
    double img0_time,seg0_time,img1_time,seg1_time;
};

list<SegImage> images;

[[noreturn]] void sync_process()
{
    cout<<std::fixed;
    cout.precision(10);
    while(true)
    {
        m_buf.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        SegImage img;
        double time = 0;

        m_buf.lock();
        if(img0_buf.empty() || img1_buf.empty() || seg0_buf.empty() || seg1_buf.empty()){
            continue;
        }

        //下面以img0的时间戳为基准，找到与img0相近的图片

        img.img0= getImageFromMsg(img0_buf.front());
        img.img0_time=img0_buf.front()->header.stamp.toSec();
        img0_buf.pop();

        img.img1_time=img1_buf.front()->header.stamp.toSec();
        if(img.img0_time+DELAY < img.img1_time){ //img0太早了
            continue;
        }
        else if(img.img1_time+DELAY < img.img0_time){ //img1太早了
            while(std::abs(img.img0_time-img.img1_time)>DELAY){
                img1_buf.pop();
                img.img1_time=img1_buf.front()->header.stamp.toSec();
            }
        }
        img.img1= getImageFromMsg(img1_buf.front());
        img1_buf.pop();


        img.seg0_time=seg0_buf.front()->header.stamp.toSec();
        if(img.img0_time+DELAY < img.seg0_time){ //img0太早了
            continue;
        }
        else if(img.seg0_time+DELAY < img.img0_time){ //seg0太早了
            while(std::abs(img.img0_time-img.seg0_time)>DELAY){
                seg0_buf.pop();
                img.seg0_time=seg0_buf.front()->header.stamp.toSec();
            }
        }
        img.seg0= getImageFromMsg(seg0_buf.front());
        seg0_buf.pop();


        img.seg1_time=seg1_buf.front()->header.stamp.toSec();
        if(img.img0_time+DELAY < img.seg1_time){ //img0太早了
            continue;
        }
        else if(img.seg1_time+DELAY < img.img0_time){ //seg1太早了
            while(std::abs(img.img0_time-img.seg1_time)>DELAY){
                seg1_buf.pop();
                img.seg1_time=seg1_buf.front()->header.stamp.toSec();
            }
        }
        img.seg1= getImageFromMsg(seg1_buf.front());
        seg1_buf.pop();

        images.push_back(img);
        cout<<(images.size())<<"\n";
        cout<<"time:\n";
        cout<<img.img0_time<<"\n";
        cout<<img.img1_time<<"\n";
        cout<<img.seg0_time<<"\n";
        cout<<img.seg1_time<<"\n";
        cout<<endl;
    }
}









int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_image_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);


    ros::Subscriber sub_img0 = n.subscribe("/cam0/image_raw", 100, img0_callback);
    ros::Subscriber sub_seg0 = n.subscribe("/cam0/segmentation", 100,seg0_callback);
    ros::Subscriber sub_img1 = n.subscribe("/cam1/image_raw", 100, img1_callback);
    ros::Subscriber sub_seg1 = n.subscribe("/cam1/segmentation", 100, seg1_callback);

    std::thread sync_thread{sync_process};

    ROS_INFO("已初始化");

    ros::spin();

    return 0;
}




