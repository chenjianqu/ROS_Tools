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
    ros::init(argc, argv, "cvt_tf_node");
    if(argc!=3){
        ROS_ERROR("usage: \nrosrun tf_test broadcast_tf target_frame source_frame");
        return -1;
    }

    ros::NodeHandle node;

    string frame1(argv[1]);
    string frame2(argv[2]);

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




    //为小数点后8位
    cout<<std::setprecision(12);

    Eigen::Isometry3d Tiso=Eigen::Isometry3d::Identity();
    transformTFToEigen(stampedTransform,Tiso);
    cout<<"T:\n"<<Tiso.matrix()<<endl<<endl;

    cout<<"translation:\n"<<Tiso.translation()<<endl<<endl;
    auto rm=Tiso.rotation();
    cout<<"rotation matrix:\n"<<rm<<endl<<endl;

    auto euler=rm.eulerAngles(2,1,0);
    cout<<"euler angles(zyx):\n"<<euler<<endl<<endl;

    Eigen::AngleAxisd angleAxisd;
    angleAxisd.fromRotationMatrix(rm);
    cout<<"angle axis:\n"<<angleAxisd.angle()<<"\n"<<angleAxisd.axis()<<endl<<endl;
    return 0;
};
