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
const string NAME_COLOR_RED = "red";
const string NAME_COLOR_BLUE = "blue";
const string NAME_COLOR_GREEN = "green";
const string NAME_COLOR_YELLOW="yellow";
const string NAME_COLOR_PINK="pink";
const string NAME_COLOR_NONE = "NO_COLOR_RECOGNIZE";
//functions to check  the quantity of points which have a certain color. It uses the hsv representation

//red color
float color_red(PCLCloudPtr cloud, int cloudSize)
    {   //definition of the pcl in hsv space
        pcl::PointXYZHSV hsv;
        // initialization of the counter
        float counter_red=0;
        // for each point of the cloud check the color
        for (int i = 0; i < cloudSize; i++)
        {   //conversion from RGB to HSV color space
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
            //check whether the point has red color
            if(hsv.h>=320){
                counter_red++;
            }
            else if(hsv.h<=40){
                counter_red++;
            }
        }
        //computing it as percentage
        counter_red=counter_red/cloudSize;
        return counter_red;

    }

//Green color

float color_green(PCLCloudPtr cloud, int cloudSize)
    {   //definition of the pcl in hsv space
        pcl::PointXYZHSV hsv;
        // initialization of the counter
        float counter_green=0;
        // for each point of the cloud check the color
        for (int i = 0; i < cloudSize; i++)
        {   //conversion from RGB to HSV color space
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
            //check whether the point has green color
            if(hsv.h<=160 && hsv.h>=80){
                counter_green++;
            }
        }
        //computing it as percentage
        counter_green=counter_green/cloudSize;
        return counter_green;
    }



//Yellow color

float color_yellow(PCLCloudPtr cloud, int cloudSize)
{   //definition of the pcl in hsv space
    pcl::PointXYZHSV hsv;
    // initialization of the counter
    float counter_yellow=0;
    // for each point of the cloud check the color
    for (int i = 0; i < cloudSize; i++)
    {   //conversion from RGB to HSV color space
        pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
        //check whether the point has yellow color
        if(hsv.h<80 && hsv.h>=40){
            counter_yellow++;
        }
    }
    //computing it as percentage
    counter_yellow=counter_yellow/cloudSize;
    return counter_yellow;
}


//Blue color
float color_blue(PCLCloudPtr cloud, int cloudSize)
    {  //definition of the pcl in hsv space
        pcl::PointXYZHSV hsv;
        // initialization of the counter
        float counter_blue=0;
        // for each point of the cloud check the color
        for (int i = 0; i < cloudSize; i++)
        {   //conversion from RGB to HSV color space
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
            //check whether the point has blue color
            if(hsv.h>160 && hsv.h<=280){
                counter_blue++;
            }
        }
        //computing it as percentage
        counter_blue=counter_blue/cloudSize;
        return counter_blue;
    }

//Pink color
float color_pink(PCLCloudPtr cloud, int cloudSize)
{   //definition of the pcl in hsv space
    pcl::PointXYZHSV hsv;
    // initialization of the counter
    float counter_pink=0;
    // for each point of the cloud check the color
    for (int i = 0; i < cloudSize; i++)
    {   //conversion from RGB to HSV color space
        pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
        //check whether the point has pink color
        if(hsv.h>280 && hsv.h<320){
            counter_pink++;
        }
    }
    //computing it as percentage
    counter_pink=counter_pink/cloudSize;
    return counter_pink;
}

// function which will be executed everytime the service is called
   bool color_info(pitt_msgs::ColorSrvMsg::Request  &req, pitt_msgs::ColorSrvMsg::Response &res)
   {
      //variable definition
       string color_name ;
       float counter_red;
       float counter_green;
       float counter_blue;
       float counter_pink;
       float counter_yellow;
       //conversion of the input PCL
       PCLCloudPtr cloud = PCManager::cloudForRosMsg(req.cloud);
       //saving the size of the cloud
       int cloudSize=cloud->points.size();
       //calling the function that computes the percentage of each color
       counter_red=color_red(cloud,cloudSize);
       counter_green=color_green(cloud,cloudSize);
       counter_blue=color_blue(cloud,cloudSize);
       counter_yellow=color_yellow(cloud,cloudSize);
       counter_pink=color_pink(cloud,cloudSize);
       // check which color is more present in each object
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
       // TODO deleta or increase with information regarding the other colors
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

