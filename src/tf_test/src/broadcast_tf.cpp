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
    ros::init(argc, argv, "broadcast_tf_node");
    if(argc<5 or argc>6){
        ROS_ERROR("usage: rosrun tf_test broadcast_tf xx.yaml keyword frameId child_frameId");
        return -1;
    }

    ros::NodeHandle node;

    string filename(argv[1]);
    string key(argv[2]);
    string frameId(argv[3]);
    string child_frameId(argv[4]);

    cv::Mat Tmat;
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened()){
        cerr<<string("无法读取文件:")+filename<<endl;
        return -1;
    }
    fs[key]>>Tmat;
    fs.release();

    Eigen::Isometry3d Teigen;
    cv::cv2eigen(Tmat,Teigen.matrix());

    if(argc==6){
        string isInverse(argv[5]);
        if(isInverse=="false"){
            cout<<"not inverse"<<endl;
        }
        else{
            cout<<"use inverse"<<endl;
            Teigen=Teigen.inverse();
        }
    }

    //根据rpy设置一个变换矩阵
    /*
    Eigen::AngleAxisd yawAxis(M_PI,Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAxis(0,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAxis(M_PI/2,Eigen::Vector3d::UnitX());
    Eigen::Quaterniond qrot;
    qrot=yawAxis*pitchAxis*rollAxis;
    Eigen::Isometry3d Trot=Eigen::Isometry3d::Identity();
    Trot.rotate(qrot);

    Teigen=Teigen * Trot;
*/


    tf::Transform Ttf;
    tf::transformEigenToTF(Teigen,Ttf);


    tf::TransformBroadcaster br;

    cout<<"broadcasting tf:\n"<<Teigen.matrix()<<endl;

    ros::Rate rate(30.0);
    while (node.ok()){
        // 发布坐标变换
        br.sendTransform(tf::StampedTransform(Ttf, ros::Time::now(), frameId, child_frameId));
        rate.sleep();
    }

    return 0;
};
