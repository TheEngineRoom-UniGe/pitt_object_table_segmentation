//
// Created by carlotta on 14/03/17.
//


#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>                             // pcl to ros conversion library
#include "pitt_msgs/ColorSrvMsg.h"                         // services and messages
#include "../point_cloud_library/pc_manager.h"				 // my static library
#include "pcl/point_types_conversion.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

using namespace ros;
using namespace pcm;
using namespace pitt_msgs;
using namespace std_msgs;
//service name
const string SRV_NAME_COLOR = "color_srv";
//color names
const string NAME_COLOR_RED = "RED";
const string NAME_COLOR_BLUE = "BLUE";
const string NAME_COLOR_GREEN = "GREEN";
const string NAME_COLOR_YELLOW="YELLOW";
const string NAME_COLOR_PINK="PINK";
const string NAME_COLOR_NONE = "NO_COLOR_RECOGNIZE";

    //functions to check the color of the point cloud starting form its average hue data

    //red color
    float color_red(PCLCloudPtr cloud, int cloudSize)
    {  pcl::PointXYZHSV hsv;
        float counter_red=0;
        for (int i = 0; i < cloudSize; i++)
        {   //conversion from RGB to HSV color space
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
            if(hsv.h>=320){
                counter_red++;
            }
            else if(hsv.h<=40){
                counter_red++;
            }
        }
        counter_red=counter_red/cloudSize;
        return counter_red;

    }

    //Green color

    float color_green(PCLCloudPtr cloud, int cloudSize)
    {
        pcl::PointXYZHSV hsv;
        float counter_green=0;
        for (int i = 0; i < cloudSize; i++)
        {   //conversion from RGB to HSV color space
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
            if(hsv.h<=160 && hsv.h>=80){
                counter_green++;
            }
        }
        counter_green=counter_green/cloudSize;
        return counter_green;
    }



//Yellow color

float color_yellow(PCLCloudPtr cloud, int cloudSize)
{
    pcl::PointXYZHSV hsv;
    float counter_yellow=0;
    for (int i = 0; i < cloudSize; i++)
    {   //conversion from RGB to HSV color space
        pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
        if(hsv.h<80 && hsv.h>=40){
            counter_yellow++;
        }
    }
    counter_yellow=counter_yellow/cloudSize;
    return counter_yellow;
}

    //Blue color
    float color_blue(PCLCloudPtr cloud, int cloudSize)
    {   pcl::PointXYZHSV hsv;
        float counter_blue=0;
        for (int i = 0; i < cloudSize; i++)
        {   //conversion from RGB to HSV color space
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
            if(hsv.h>160 && hsv.h<=280){
                counter_blue++;
            }
        }
        counter_blue=counter_blue/cloudSize;
        return counter_blue;
    }
//Pink color
float color_pink(PCLCloudPtr cloud, int cloudSize)
{   pcl::PointXYZHSV hsv;
    float counter_pink=0;
    for (int i = 0; i < cloudSize; i++)
    {   //conversion from RGB to HSV color space
        pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
        if(hsv.h>280 && hsv.h<320){
            counter_pink++;
        }
    }
    counter_pink=counter_pink/cloudSize;
    return counter_pink;
}


   bool color_info(pitt_msgs::ColorSrvMsg::Request  &req, pitt_msgs::ColorSrvMsg::Response &res)
   {
      //variable definition
       string color_name ;
       float counter_red;
       float counter_green;
       float counter_blue;
       float counter_pink;
       float counter_yellow;
       pcl::PointXYZHSV hsv;
       //conversion of the input PCL
       PCLCloudPtr cloud = PCManager::cloudForRosMsg(req.cloud);
       int cloudSize=cloud->points.size();
       counter_red=color_red(cloud,cloudSize);
       counter_green=color_green(cloud,cloudSize);
       counter_blue=color_blue(cloud,cloudSize);
       counter_yellow=color_yellow(cloud,cloudSize);
       counter_pink=color_pink(cloud,cloudSize);
       // check which color is the point cloud

        if(counter_green>counter_red && counter_green>counter_blue && counter_green>counter_pink && counter_green>counter_yellow)
       {
           color_name=NAME_COLOR_GREEN;
       }
       else if(counter_blue>counter_red && counter_blue>counter_green && counter_blue>counter_pink && counter_blue>counter_yellow)
       {
           color_name=NAME_COLOR_BLUE;
       }
       else if(counter_red>counter_blue && counter_red>counter_green && counter_red>counter_yellow && counter_red>counter_pink)
        {
            color_name = NAME_COLOR_RED;
        }
        else if(counter_pink>counter_blue && counter_pink>counter_green && counter_pink>counter_yellow && counter_pink>counter_red)
       {
           color_name=NAME_COLOR_PINK;
       }
       else if (counter_yellow>counter_blue && counter_yellow>counter_green && counter_yellow>counter_pink && counter_yellow>counter_red){
           color_name=NAME_COLOR_YELLOW;
       }
       else
       {
           color_name = NAME_COLOR_NONE;
       }

       //filling the response
       res.Color.data=color_name;
       res.bluePercentage.data=counter_blue;
       res.greenPercentage.data=counter_green;
       res.redPercentage.data=counter_red;

       return true;
   }
int main(int argc, char **argv)
{
    ros::init(argc, argv,SRV_NAME_COLOR );
    ros::NodeHandle n;
    ros::ServiceServer service =n.advertiseService(SRV_NAME_COLOR,color_info);
    ros::spin();
    return 0;
}

