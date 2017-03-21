//
// Created by carlotta on 14/03/17.
//


#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>                             // pcl to ros conversion library
#include "pitt_msgs/ColorSrvMsg.h"                         // services and messages
#include "../point_cloud_library/pc_manager.h"				 // my static library

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
 string SRV_NAME_COLOR = "color_srv";
const string NAME_COLOR_RED = "red";
const string NAME_COLOR_BLUE = "blue";
const string NAME_COLOR_GREEN = "green";
const string NAME_COLOR_NONE = "none";

    //functions to check the color of the point cloud starting form its RGB data
    //red color
    bool color_red(float *red,float *green,float *blue)
    {
        if(*red>200 && *green<50 && *blue<50){
            return true;
        }
        else {

            return false;
        }
    }
    //Green color

    bool color_green(float *red,float *green,float *blue)
    {
        if(*green>200 && *red<50 && *blue<50){
            return true;
        }
        else {

            return false;
        }
    }

    //Blue color
    bool color_blue(float *red,float *green,float *blue)
    {
        if(*blue>200 && *red<50 && *green<50){
            return true;
        }
        else {

            return false;
        }
    }
  // function to compute the average RGB value
   void average_color(PCLCloudPtrRGB& cloud,float* red,float* green,float*  blue)
   {   float r,g,b;
       int cloudSize=cloud->points.size();
       r=0;
       g=0;
       b=0;

       for (int i = 0; i < cloudSize; i++)
       {
           r = r+cloud->points[i].r;
           g = g+cloud->points[i].g;
           b = b+cloud->points[i].b;
       }
       *red=r/cloudSize;
       *green=g/cloudSize;
       *blue=b/cloudSize;

   }
   bool color_info(pitt_msgs::ColorSrvMsg::Request  &req, pitt_msgs::ColorSrvMsg::Response &res)
   {
      //variable definition
       string color_name ;
       float red, blue, green;
       //conversion of the input PCL
       PCLCloudPtrRGB cloud = cloudForRosMsgRGB(req.cloud);
       //computation of the average RGB value
       average_color(cloud,&red,&green,&blue);
       // check which color is the point cloud
       if(color_red(&red, &green,&blue))
       {
           color_name=NAME_COLOR_RED;
       }
       else if(color_green(&red, &green,&blue))
       {
           color_name=NAME_COLOR_GREEN;
       }
       else if(color_blue(&red, &green,&blue))
       {
           color_name=NAME_COLOR_BLUE;
       }
       else
       {
           color_name = NAME_COLOR_NONE;
       }
       //filling the response
       res.color.data=color_name;


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

