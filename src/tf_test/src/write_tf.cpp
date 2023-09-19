#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <turtlesim/Pose.h>
#include <tf_conversions/tf_eigen.h>



using namespace std;

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "write_tf_node");
    if(argc!=4){
        ROS_ERROR("usage: \nrosrun tf_test broadcast_tf target_frame source_frame xx.yaml");
        return -1;
    }

    ros::NodeHandle node;

    string frame1(argv[1]);
    string frame2(argv[2]);
    string filename(argv[3]);


    tf::TransformListener listener;


    ros::Rate rate(10.0);
    tf::StampedTransform stampedTransform;
    try{
        // 查找frame1与frame2的坐标变换
        listener.waitForTransform(frame1, frame2, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform(frame1, frame2, ros::Time(0), stampedTransform);
    }
    catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return 1;
    }


    /*
    auto origin=stampedTransform.getOrigin();
    auto rotation=stampedTransform.getRotation();
    //转换为Eigen，在转换为Mat
    Eigen::Vector3d t(origin.x(),origin.y(),origin.z());
    Eigen::Quaterniond q(rotation.w(),rotation.x(),rotation.y(),rotation.z());

    cout<<"origin:\n"<<t<<endl;
    cout<<"rotation:\n"<<q.matrix()<<endl;

    Eigen::Affine3d Taff=Eigen::Affine3d::Identity();
    Taff.rotate(q);
    Taff.translate(t);
    */

    Eigen::Isometry3d Tiso=Eigen::Isometry3d::Identity();
    transformTFToEigen(stampedTransform,Tiso);
    cout<<"T:\n"<<Tiso.matrix()<<endl;

    cv::Mat mat;
    cv::eigen2cv(Tiso.matrix(),mat);

    //写入矩阵
    cv::FileStorage fs(filename,cv::FileStorage::WRITE);
    fs<<"tf"<<mat;
    fs.release();

    cout<<"写入: "<<filename<<" 完成"<<endl;

    return 0;
};
