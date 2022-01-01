//
// Created by chen on 2021/8/27.
//
//
// Created by chen on 2021/8/27.
//


#include<string>
#include <thread>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>



using namespace std;



void BuildLineStripMarker(visualization_msgs::Marker &msg){

    msg.header.frame_id="base_link";
    msg.header.stamp=ros::Time::now();
    msg.ns="box_strip";
    msg.action=visualization_msgs::Marker::ADD;
    msg.pose.orientation.w=1.0;

    //暂时使用类别代替这个ID
    msg.id=0;//当存在多个marker时用于标志出来
    //cout<<msg.id<<endl;
    msg.lifetime=ros::Duration(4);//持续时间3s，若为ros::Duration()表示一直持续

    msg.type=visualization_msgs::Marker::LINE_STRIP;//marker的类型
    msg.scale.x=0.01;//线宽
    msg.color.r=1.0;msg.color.g=1.0;msg.color.b=1.0;
    msg.color.a=1.0;//不透明度

    //设置立方体的八个顶点
    geometry_msgs::Point minPt,maxPt;
    minPt.x=0;minPt.y=0;minPt.z=0;
    maxPt.x=1;maxPt.y=1;maxPt.z=1;
    geometry_msgs::Point p[8];
    p[0].x=minPt.x;p[0].y=minPt.y;p[0].z=minPt.z;
    p[1].x=maxPt.x;p[1].y=minPt.y;p[1].z=minPt.z;
    p[2].x=maxPt.x;p[2].y=minPt.y;p[2].z=maxPt.z;
    p[3].x=minPt.x;p[3].y=minPt.y;p[3].z=maxPt.z;
    p[4].x=minPt.x;p[4].y=maxPt.y;p[4].z=maxPt.z;
    p[5].x=maxPt.x;p[5].y=maxPt.y;p[5].z=maxPt.z;
    p[6].x=maxPt.x;p[6].y=maxPt.y;p[6].z=minPt.z;
    p[7].x=minPt.x;p[7].y=maxPt.y;p[7].z=minPt.z;

    //这个类型仅将相邻点进行连线
    for(auto &pt : p)
        msg.points.push_back(pt);
    //为了保证矩形框的其它边存在：
    msg.points.push_back(p[0]);
    msg.points.push_back(p[3]);
    msg.points.push_back(p[2]);
    msg.points.push_back(p[5]);
    msg.points.push_back(p[6]);
    msg.points.push_back(p[1]);
    msg.points.push_back(p[0]);
    msg.points.push_back(p[7]);
    msg.points.push_back(p[4]);
}



void BuildTextMarker(visualization_msgs::Marker &msg)
{
    msg.header.frame_id="base_link";
    msg.header.stamp=ros::Time::now();
    msg.ns="box_text";
    msg.action=visualization_msgs::Marker::ADD;

    //暂时使用类别代替这个ID
    msg.id=0+1000;//当存在多个marker时用于标志出来
    msg.lifetime=ros::Duration(4);//持续时间4s，若为ros::Duration()表示一直持续

    msg.type=visualization_msgs::Marker::TEXT_VIEW_FACING;//marker的类型
//    msg.scale.x=0.01;//线宽
    msg.scale.z=0.1;
    msg.color.r=1.0;msg.color.g=1.0;msg.color.b=1.0;
    msg.color.a=1.0;//不透明度

    geometry_msgs::Point minPt,maxPt;
    minPt.x=0;minPt.y=0;minPt.z=0;
    maxPt.x=1;maxPt.y=1;maxPt.z=1;

    geometry_msgs::Pose pose;
    pose.position.x=maxPt.x;pose.position.y=maxPt.y;pose.position.z=maxPt.z;
    pose.orientation.w=1.0;
    //msg.pose=pose;

    msg.text="test";
}




void BuildArrowMarker(visualization_msgs::Marker &msg,double start_x,double start_y,double start_z,double end_x,double end_y,double end_z)
{
    msg.header.frame_id="world";
    msg.header.stamp=ros::Time::now();
    msg.ns="arrow_strip";
    msg.action=visualization_msgs::Marker::ADD;
    msg.pose.orientation.w=1.0;

    //暂时使用类别代替这个ID
    msg.id=0;//当存在多个marker时用于标志出来
    //cout<<msg.id<<endl;
    msg.lifetime=ros::Duration();//持续时间3s，若为ros::Duration()表示一直持续

    msg.type=visualization_msgs::Marker::LINE_STRIP;//marker的类型
    msg.scale.x=0.01;//线宽
    msg.color.r=1.0;msg.color.g=1.0;msg.color.b=1.0;
    msg.color.a=1.0;//不透明度

    //设置立方体的八个顶点
    geometry_msgs::Point start,end;
    start.x=start_x;start.y=start_y;start.z=start_z;
    end.x=end_x;end.y=end_y;end.z=end_z;

    const double arrow_len=std::sqrt((start.x-end.x) * (start.x-end.x) +(start.y-end.y) * (start.y-end.y) + (start.z-end.z) * (start.z-end.z)) /8.;

    geometry_msgs::Point p[7];
    p[0]=start;
    p[1]=end;
    p[2]=end;
    if(start.x < end.x){
        p[2].x -= arrow_len;
    }else{
        p[2].x += arrow_len;
    }
    p[3]=end;
    p[4]=end;
    if(start.y < end.y){
        p[4].y -= arrow_len;
    }else{
        p[4].y += arrow_len;
    }
    p[5]=end;
    p[6]=end;
    if(start.z < end.z){
        p[6].z -= arrow_len;
    }else{
        p[6].z += arrow_len;
    }

    //这个类型仅将相邻点进行连线
    for(auto &pt : p)
        msg.points.push_back(pt);
}








int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");//防止中文乱码
    ros::init(argc, argv, "detect_node");
    ros::start();
    ros::NodeHandle nh;

    ros::Publisher obj_pub=nh.advertise<visualization_msgs::MarkerArray>("marker_test_topic",10);

    cout<<"loop:"<<endl;
    int counter=0;

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker lineStripMarker,textMarker;
        //BuildLineStripMarker(lineStripMarker);
        //BuildTextMarker(textMarker);
        BuildArrowMarker(lineStripMarker,0,0,0,0,1,1);
        markers.markers.push_back(lineStripMarker);
        //markers.markers.push_back(textMarker);
        obj_pub.publish(markers);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    ROS_INFO_STREAM("程序结束");
    return 0;
}




