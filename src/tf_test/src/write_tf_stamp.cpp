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

using std::string;
using std::endl;
using std::cerr;



/**
 * 将字符串写入到文件中
 * @param path
 * @param text
 */
void WriteTextFile(const string& path,const std::string& text){
    static std::set<string> path_set;
    ///第一次,清空文件
    if(path_set.find(path)==path_set.end()){
        path_set.insert(path);
        std::ofstream fout( path.data(), std::ios::out);
        fout.close();
    }

    ///添加到文件后面
    std::ofstream fout(path.data(), std::ios::app);
    fout<<text<<endl;
    fout.close();
}




int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "write_tf_node");
    if(argc!=4){
        ROS_ERROR("usage: rosrun tf_test write_tf_stamp target_frame source_frame xx.txt");
        return -1;
    }

    ros::NodeHandle node;

    string frame1(argv[1]);
    string frame2(argv[2]);
    string filename(argv[3]);

    tf::TransformListener listener;

    ros::Rate rate(10.0);

    tf::StampedTransform transform;

    while(ros::ok()){
        try{
            /// 查找frame1与frame2的坐标变换
            listener.waitForTransform(frame1, frame2, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(frame1, frame2, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex){
            cerr<<ex.what()<<endl;
            return 1;
        }

        string line = std::to_string(ros::Time::now().toSec()) + " " +
                std::to_string(transform.getOrigin().x()) + " " +
                std::to_string(transform.getOrigin().y()) + " " +
                std::to_string(transform.getOrigin().z()) + " " +
                std::to_string(transform.getRotation().x()) + " " +
                std::to_string(transform.getRotation().y()) + " " +
                std::to_string(transform.getRotation().z()) + " " +
                std::to_string(transform.getRotation().w());

        WriteTextFile(filename,line);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
};
