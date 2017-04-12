//
// Created by carlotta on 14/03/17.
//


#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>                             // pcl to ros conversion library
#include "pitt_msgs/ColorSrvMsg.h"                         // services and messages
#include "../point_cloud_library/pc_manager.h"				 // my static library
#include "pcl/point_types_conversion.h"

//definition of the type PCL with RGB information

typedef pcl::PointCloud< pcl::PointXYZRGB>::Ptr PCLCloudPtrRGB;          // for point cloud smart pointer
typedef pcl::PointCloud< pcl::PointXYZRGB> PCLCloudRGB;						// for point cloud

//to convert ros msg in pcl point cloud with RGB information

PCLCloudPtrRGB cloudForRosMsgRGB( PointCloud2Ptr input){
    PCLCloudPtrRGB cl( new PCLCloudRGB);
    fromROSMsg ( *input, *cl);
    return( cl);
}

PCLCloudPtrRGB cloudForRosMsgRGB ( PointCloud2 input){
    PCLCloudPtrRGB cl( new PCLCloudRGB);
    fromROSMsg ( input, *cl);
    return( cl);
}

//service name
const string SRV_NAME_COLOR = "color_srv";
const string NAME_COLOR_RED = "RED";
const string NAME_COLOR_BLUE = "BLUE";
const string NAME_COLOR_GREEN = "GREEN";
const string NAME_COLOR_NONE = "NO_COLOR_RECOGNIZE";

    //functions to check the color of the point cloud starting form its RGB data

    //red color
    bool color_red(float  hAverage)
    {
        if(hAverage > 140){
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
   {    ROS_INFO_STREAM("looking for colors ..."<<endl);
      //variable definition
       string color_name ;
       float hAverage=0;
       pcl::PointXYZHSV hsv;
       //conversion of the input PCL
       PCLCloudPtrRGB cloud = cloudForRosMsgRGB(req.cloud);
       int cloudSize=cloud->points.size();
       //computation of the average HSV value
       for (int i = 0; i < cloudSize; i++)
       {
           pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
           hAverage=hAverage+hsv.h;
       }

       hAverage=hAverage/cloudSize;
       ROS_INFO("average color");
       ROS_INFO("%d",hAverage);
       // check which color is the point cloud
       if(color_red(hAverage))
       {
           color_name=NAME_COLOR_RED;
           ROS_INFO_STREAM("RED"<<endl);
       }
       else if(color_green(hAverage))
       {
           color_name=NAME_COLOR_GREEN;
           ROS_INFO_STREAM("GREEN"<<endl);
       }
       else if(color_blue(hAverage))
       {
           color_name=NAME_COLOR_BLUE;
           ROS_INFO_STREAM("BLUE"<<endl);
       }
       else
       {
           color_name = NAME_COLOR_NONE;

       }
       //filling the response
       res.color.data=color_name;
       //res.Hue.data=hAverage;

       return true;
   }
int main(int argc, char **argv)
{
    ros::init(argc, argv,SRV_NAME_COLOR );
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService(SRV_NAME_COLOR,color_info);
    ROS_INFO("Initialized Color Service color information");
    ros::spin();

    return 0;
}

