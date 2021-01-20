#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h> //for sonar data
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream>
#include <Aria/Aria.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> // can optionally publish sonar as new type pointcloud2
#include <nav_msgs/Odometry.h>
#include "rosaria/BumperState.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include <rosaria/RosAriaConfig.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "tf/transform_datatypes.h"
#include <sstream>
#include <stdlib.h>
#include<final/m1.h>
#define PI 3.14159265
#define SONAR_NUM 8
using namespace std;
float offset[SONAR_NUM] ={90, 115, 141, 167, 167, 141, 115, 90};

//sensor_msgs::PointCloud sonarData;
int seq = 0;
float x=0.5;
bool human=false;
float distToObstacex[SONAR_NUM],distToObstacey[SONAR_NUM];
ros::Publisher robot;
geometry_msgs::Twist cmdvel;
int noworient;
void humanCallback(final::m1 m1){
    human=true;
    x=m1.x;
    //printf(%f,x);


}
bool getoritation(int rotangle,int origentangle) //intput rotate angle
    {
        int frotangle=0;
        //ROS_INFO("46116515311");
        for(int i = 0; i < 2; i++)
        {
            cmdvel.angular.x = 0;
            cmdvel.angular.y = 0;
            cmdvel.angular.z = 0;
            robot.publish(cmdvel);
        }
        frotangle=(origentangle+rotangle)%360;
        while(ros::ok() && rotangle>0 && noworient<frotangle-10)
        {  
            cmdvel.angular.z = 1 * 3.14/6;
            robot.publish(cmdvel);               
            ros::spinOnce();
            //ROS_INFO("123");
            //rate.sleep();
            ros::Duration(500);
        }
        while(ros::ok() && rotangle<0 && noworient<frotangle+10)
        {  
            cmdvel.angular.z = -1 * 3.14/10;
            robot.publish(cmdvel);               
            ros::spinOnce();
            //ROS_INFO("%.2f",noworient);
            //rate.sleep();
            ros::Duration(500);
        }
    for(int i = 0; i < 2; i++)
        {
            cmdvel.angular.x = 0;
            cmdvel.angular.y = 0;
            cmdvel.angular.z = 0;
            robot.publish(cmdvel);
        }
    return true;
}
void get_sonarData_Callback(const sensor_msgs::PointCloud::ConstPtr &sonarScanedData)
{
    float tmpX = 0.0, tmpY=0.0;
    seq = sonarScanedData->header.seq;

    //printf("seq of sonar beam  and  distance measured-->\n");
    //printf("Frame[%d] :  \n", seq);

    printf("\n");

    for (int i=0; i<SONAR_NUM; i++)    
    {
        tmpX= sonarScanedData->points[i].x; //coordinate x
        tmpY= sonarScanedData->points[i].y; //coordinate y
        //printf("%f,%f\n",tmpX,tmpY);
        //distToObstace[i] = int(sqrt(tmpX*tmpX+tmpY*tmpY)*1000);//offset[i]);
        distToObstacex[i] = (tmpX*1000);//offset[i]);
        distToObstacey[i] = (tmpY*1000);
        //printf("num = %d ,distance = %d\n",offset[3],distToObstace[3]);
        printf("num = %f ,distance = %f\n",offset[i],distToObstacex[i]);
    
    }
    //printf("%f,%f\n",tmpX,tmpY);
    printf("\n\n");
}
float nowx,nowy;
void poseMessageReceived(const nav_msgs::Odometry& msg) 
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation,quat);
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    noworient=int(rpy.z/3.14*180);
    if (noworient<0){noworient+=360;}
    nowx=float(msg.pose.pose.position.x);
    nowy=float(msg.pose.pose.position.y);
    
	printf("Current direction= %d\n" ,noworient);
	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sonar");
    ros::NodeHandle n;
    ros::Publisher robot;
    geometry_msgs::Twist cmdvel;
    //plsease modify the topic name “/RosAria/sonar” according to the robot connected
    ros::Subscriber get_sonar_data = n.subscribe<sensor_msgs::PointCloud>("/RosAria/sonar", 100, get_sonarData_Callback);
    ros::Subscriber pose = n.subscribe("RosAria/pose", 1000, &poseMessageReceived) ; //supply pose
    ros::Subscriber gethuman = n.subscribe<final::m1>("chatter", 100, humanCallback);
    robot = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1000);
    printf("\n********** Sonar Readings: *********\n");
    //cmdvel.linear.x=0.0;
    //cmdvel.angular.z=0.0;
    //robot.publish(cmdvel);
    printf("123");_
    while (ros::ok())
    { 
        switch(human){
            case true:
                if(x>=0.4 && x<=0.6){cmdvel.angular.z=0;}
                else{cmdvel.angular.z=-(x-0.5)/2;}
                if(distToObstacex[2]>800 && distToObstacex[3]>800 && distToObstacex[4]>800 && distToObstacex[5]>800){
                    cmdvel.linear.x=((distToObstacex[2]+distToObstacex[3]+distToObstacex[4]+distToObstacex[5])-3200)/500;
                }
                else{cmdvel.linear.x=0;}
            case false:
                int targetangle = atan((x-0.5)*2*tan(37*PI/180));
                getoritation(targetangle,noworient);
                float targetdistance= (distToObstacex[2]+distToObstacex[3]+distToObstacex[4]+distToObstacex[5])/4;
                float targetx= targetdistance/1000*cos(noworient*PI/180);
                float targety = targetdistance/1000*sin(noworient*PI/180);
                while(!human){
                    printf("cant find human\n");

                }
                


        }
        /*
        printf("%f",x);
        if(!human){
        cmdvel.angular.z=0.03;
        cmdvel.linear.x=0.0;
    }
    else{
    if(distToObstacex[1]>1000 && distToObstacex[2]>1000 && distToObstacex[3]>1000 && distToObstacex[4]>1000 && distToObstacex[5]>1000 && distToObstacex[6]>1000){
    cmdvel.linear.x=0.1;
    }
    if(x<=0.4){cmdvel.angular.z=0.1;}
    if(x>0.4 && x<0.6){cmdvel.angular.z=0.0;}
    if(x>=0.6){cmdvel.angular.z=-0.1;}

    }*/
       
        robot.publish(cmdvel);
        ros::Duration(0.2).sleep();
        
    /*ROS message callback processing function. The two of them usually appear in the main loop of ROS. The program needs to call ros::spin() or ros::spinOnce() continuously. The difference between the two is that the former will not return after the call, that is, your main program is here. Do not proceed further, and the latter can continue to execute the subsequent program after the call*/
    //  https://www.cnblogs.com/liu-fa/p/5925381.html
    ros::spinOnce();
}
return 0;
}