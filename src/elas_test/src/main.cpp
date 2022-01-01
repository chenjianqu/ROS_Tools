//
// Created by chen on 2021/10/26.
//

#include <cstdio>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <chrono>

#include <stereo_msgs/DisparityImage.h>


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <libelas/elas.h>

using namespace std;

using PointCloud=pcl::PointCloud<pcl::PointXYZRGB>;


const float baseline=0.05f;

class TicToc{
public:
    TicToc(){
        tic();
    }
    void tic(){
        start = std::chrono::system_clock::now();
    }
    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
    void toc_print_tic(const char* str){
        printf("%s : %f ms\n",str,toc());
        tic();
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};




struct SegImage{
    cv::Mat color0,seg0,color1,seg1;
    double time0,seg0_time,time1,seg1_time;
    cv::Mat gray0,gray1;

    void setGrayImage(){
        cv::cvtColor(color0, gray0, CV_BGR2GRAY);
        if(!color1.empty())
            cv::cvtColor(color1, gray1, CV_BGR2GRAY);
    }
};


class Camera{
public:
    using Ptr=std::shared_ptr<Camera>;
    Camera(float fx_,float fy_,float cx_,float cy_):fx(fx_),fy(fy_),cx(cx_),cy(cy_)
    {
    }
    void pixel2Cam(float u,float v,float disp_value,cv::Point3f &point) const{
        float x=(u-cx)/fx;
        float y=(v-cy)/fy;
        point.z=fx * baseline / disp_value;
        point.x=x*point.z;
        point.y=y*point.z;
    }

    float fx,fy,cx,cy;
    float k1,k2,p1,p2;
    int width,heigh;
};



const int QUEUE_SIZE=50;
const double DELAY=0.005;






queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> seg0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<sensor_msgs::ImageConstPtr> seg1_buf;
std::mutex m_buf;

std::shared_ptr<Elas> elas;
Elas::parameters elas_param;
Camera::Ptr camera;
ros::Publisher pub_instance_pointcloud;


void calDisparityImage(SegImage &img,cv::Mat &disp)
{
    const int width=img.gray0.cols;
    const int height=img.gray0.rows;

    uint8_t  *l_image_data,*r_image_data;
    l_image_data=img.gray0.data;
    r_image_data=img.gray1.data;

    float *l_disp_data=reinterpret_cast<float *>(&disp.data[0]);
    float *r_disp_data=new float[width * height * sizeof(float)];

    int32_t step=img.gray0.step;
    const int32_t dims[3]={width,height,step};

    cout<<"计算视差"<<endl;

    elas->process(l_image_data,r_image_data,l_disp_data,r_disp_data,dims);
    delete r_disp_data;
}


void calPointCloud(cv::Mat &disp,cv::Mat &color,PointCloud::Ptr cloud)
{
    cout<<"计算点云"<<endl;

    for(int i=0;i<disp.rows;++i){
        for(int j=0;j<disp.cols;++j){
            float d=disp.at<float>(i,j);
/*限制深度1-50m,则限制视差值:
 * 0.2=376 * 0.05 / v ; v=94
 * 50=376 * 0.05 /v ; v=0.376
 * */
            if(d<=0.376 || d>=94)
                continue;
            cv::Point3f pt;
            camera->pixel2Cam(j,i,d,pt);
            pcl::PointXYZRGB point;
            point.x=pt.x;
            point.y=pt.y;
            point.z=pt.z;
            cv::Vec3b bgr=color.at<cv::Vec3b>(i,j);
            point.r=bgr[2];
            point.g=bgr[1];
            point.b=bgr[0];
            cloud->push_back(point);
        }
    }
}




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


inline cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr= cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    return ptr->image.clone();
}



[[noreturn]] void sync_process()
{
    cout<<std::fixed;
    cout.precision(10);

    size_t cnt=0;
    while(true)
    {
        m_buf.lock();
        //等待图片
        if(img0_buf.empty() || img1_buf.empty() || seg0_buf.empty() || seg1_buf.empty()){
            m_buf.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        ///下面以img0的时间戳为基准，找到与img0相近的图片
        SegImage img;
        img.color0= getImageFromMsg(img0_buf.front());
        img.time0=img0_buf.front()->header.stamp.toSec();
        img0_buf.pop();

        img.time1=img1_buf.front()->header.stamp.toSec();
        if(img.time0 + DELAY < img.time1){ //img0太早了
            m_buf.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        else if(img.time1 + DELAY < img.time0){ //img1太早了
            while(std::abs(img.time0 - img.time1) > DELAY){
                img1_buf.pop();
                img.time1=img1_buf.front()->header.stamp.toSec();
            }
        }
        img.color1= getImageFromMsg(img1_buf.front());
        img1_buf.pop();


        img.seg0_time=seg0_buf.front()->header.stamp.toSec();
        if(img.time0 + DELAY < img.seg0_time){ //img0太早了
            m_buf.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        else if(img.seg0_time+DELAY < img.time0){ //seg0太早了
            while(std::abs(img.time0 - img.seg0_time) > DELAY){
                seg0_buf.pop();
                img.seg0_time=seg0_buf.front()->header.stamp.toSec();
            }
        }
        img.seg0= getImageFromMsg(seg0_buf.front());
        seg0_buf.pop();


        img.seg1_time=seg1_buf.front()->header.stamp.toSec();
        if(img.time0 + DELAY < img.seg1_time){ //img0太早了
            m_buf.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        else if(img.seg1_time+DELAY < img.time0){ //seg1太早了
            while(std::abs(img.time0 - img.seg1_time) > DELAY){
                seg1_buf.pop();
                img.seg1_time=seg1_buf.front()->header.stamp.toSec();
            }
        }
        img.seg1= getImageFromMsg(seg1_buf.front());
        seg1_buf.pop();

        m_buf.unlock();


        ///rgb to gray
        img.setGrayImage();
        cnt++;

        if(cnt%5!=0)
            continue;

        ///计算视差图
        TicToc ticToc;
        cv::Mat disp(img.gray0.rows,img.gray0.cols,CV_32F);
        calDisparityImage(img,disp);
        ticToc.toc_print_tic("计算视差:");
       //std::this_thread::sleep_for(std::chrono::milliseconds(10));

        PointCloud::Ptr cloud(new PointCloud);
        calPointCloud(disp,img.color0,cloud);

        ticToc.toc_print_tic("计算点云:");

        if(cloud->size()>0){
            std_msgs::Header header;
            header.stamp = ros::Time(img.time0);
            header.frame_id="world";
            sensor_msgs::PointCloud2 point_cloud_msg;
            pcl::toROSMsg(*cloud,point_cloud_msg);
            point_cloud_msg.header = header;
            pub_instance_pointcloud.publish(point_cloud_msg);
        }

        ticToc.toc_print_tic("发布点云:");

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "elas_test");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    elas_param.disp_min=0;
    elas_param.disp_max=255;
    elas_param.support_threshold=0.95f;
    elas_param.support_texture=10;
    elas_param.candidate_stepsize=5;
    elas_param.incon_window_size=5;
    elas_param.incon_threshold=5;
    elas_param.incon_min_support=5;
    elas_param.add_corners=false;
    elas_param.grid_size=20;
    elas_param.beta=0.02;
    elas_param.gamma=3;
    elas_param.sigma=1;
    elas_param.sradius=2;
    elas_param.match_texture=1;
    elas_param.lr_threshold=2;
    elas_param.speckle_sim_threshold=1;
    elas_param.speckle_size=200;
    elas_param.ipol_gap_width=300;
    elas_param.filter_median=false;
    elas_param.filter_adaptive_mean= true;
    elas_param.postprocess_only_left=true;
    elas_param.subsampling= false;

    elas=std::make_shared<Elas>(elas_param);
    camera=std::make_shared<Camera>(376.,376.,376.,240.);


    pub_instance_pointcloud=n.advertise<sensor_msgs::PointCloud2>("stereo_point_cloud", 100);


    ROS_WARN("waiting for image...");


    ros::Subscriber sub_img0 = n.subscribe("/cam0/image_raw", 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe("/cam1/image_raw", 100, img1_callback);
    ros::Subscriber sub_seg0 = n.subscribe("/cam0/segmentation", 100, seg0_callback);
    ros::Subscriber sub_seg1 = n.subscribe("/cam1/segmentation", 100, seg1_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}




