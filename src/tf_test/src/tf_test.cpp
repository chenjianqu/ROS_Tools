#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <turtlesim/Pose.h>
std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    // tf广播器
    static tf::TransformBroadcaster br;
    // 根据乌龟当前的位姿，设置相对于世界坐标系的坐标变换
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    // 发布坐标变换
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "my_tf_test");
    ros::NodeHandle node;


    tf::TransformListener listener;
	// tf广播器
    static tf::TransformBroadcaster br;



    ros::Rate rate(10.0);
    while (node.ok()){
        tf::StampedTransform map2a,a2b;
        try{
            // 查找turtleA与turtleB的坐标变换
            listener.waitForTransform("/map", "/a", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/map", "/a", ros::Time(0), map2a);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }


	try{
            // 查找turtleA与turtleB的坐标变换
            listener.waitForTransform("/a", "/b", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/a", "/b", ros::Time(0), a2b);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

   
	    tf::Transform map2c;
	map2c.mult(map2a,a2b);
    // 发布坐标变换
    br.sendTransform(tf::StampedTransform(map2c, ros::Time::now(), "map", "c"));


        rate.sleep();
    }
    return 0;
};
