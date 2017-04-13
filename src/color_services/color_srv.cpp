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
const string NAME_COLOR_NONE = "NO_COLOR_RECOGNIZE";

    //functions to check the color of the point cloud starting form its average hue data

    //red color
    int color_red(PCLCloudPtr cloud, int cloud_size)
    {  pcl::PointXYZHSV hsv;
        if(hAverage > 180){
            return true;
        }
        else {

            return false;
        }
    }
bool color_yellow(float  hAverage)
{
    if(hAverage > 140 && hAverage<180){
        return true;
    }
    else {

        return false;
    }
}
    //Green color

    bool color_green(float  hAverage)
    {
        if(hAverage < 150 && hAverage > 100){
            return true;
        }
        else {

            return false;
        }
    }

    //Blue color
    bool color_blue(float  hAverage)
    {
        if(hAverage > 200){
            return true;
        }
        else {

            return false;
        }
    }

   bool color_info(pitt_msgs::ColorSrvMsg::Request  &req, pitt_msgs::ColorSrvMsg::Response &res)
   {
      //variable definition
       string color_name ;
       float hAverage=0;
       float sum_sq=0;
       float variance=0;
       pcl::PointXYZHSV hsv;
       //conversion of the input PCL
       PCLCloudPtr cloud = PCManager::cloudForRosMsg(req.cloud);
       int cloudSize=cloud->points.size();

       //computation of the average HSV value
       for (int i = 0; i < cloudSize; i++)
       {   //conversion from RGB to HSV color space
           pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
           hAverage=hAverage+hsv.h;
           sum_sq=sum_sq+hsv.h*hsv.h;
           ROS_INFO_STREAM(hsv.h<<endl);
       }

       hAverage=hAverage/cloudSize;
       variance=(sum_sq-(hAverage*hAverage)/cloudSize)/(cloudSize-1);

       // check which color is the point cloud
       if(color_red(hAverage))
       {
           color_name=NAME_COLOR_RED;
       }
       else if(color_green(hAverage))
       {
           color_name=NAME_COLOR_GREEN;
       }
       else if(color_blue(hAverage))
       {
           color_name=NAME_COLOR_BLUE;
       }
       else if(color_yellow(hAverage))
       {
           color_name=NAME_COLOR_YELLOW;
       }
       else
       {
           color_name = NAME_COLOR_NONE;
       }

       //filling the response
       res.Color.data=color_name;
       res.Hue.data=hAverage;
       res.Variance.data=variance;
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

