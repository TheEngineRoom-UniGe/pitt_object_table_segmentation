#include "ros/ros.h"
#include <iostream>

// services and messages
#include "pitt_msgs/DeepFilter.h"

// custom class
#include "../point_cloud_library/pc_manager.h"
#include "../point_cloud_library/srv_manager.h"

// used name space
//using namespace pitt_object_table_segmentation;
using namespace pitt_msgs;
using namespace ros;
using namespace pcm;
using namespace pcl;
using namespace srvm;

// default ros parameter values


// used abbreviations (typedef: PCLCloud, PCLCloudPtr, PCLNormal, PCLNormalPtr ) (given from PCManager in turn given from PCPrimitive)

// remove NaN from an input cloud and return two separate clouds (close & far).
// close contains all points below the given deep threshold
// far contains all points above the given deep threshold
bool deepFiltering( DeepFilter::Request& req, DeepFilter::Response& res){
	// get service inputs
	PCLCloudPtr cloud = PCManager::cloudForRosMsg( req.input_cloud);

	// set parameter default if <0
	float thDeep = srvm::getServiceFloatParameter( req.deep_threshold, srvm::DEFAULT_PARAM_SRV_DEEP_FILTER_DEPTH_THRESHOLD);
	// initialize outputs
	PCLCloudPtr cloudCloser ( new PCLCloud);
	PCLCloudPtr cloudFurther ( new PCLCloud);

	// points iteration   (((  FILTERING   )))
	for( int i = 0; i < cloud->points.size(); i++){
		if( cloud->points[ i].z == cloud->points[ i].z){ // then is not NaN
			if( cloud->points[ i].z > thDeep)
				cloudFurther->points.push_back( cloud->points[ i]);
			else cloudCloser->points.push_back( cloud->points[ i]);
		}
	}

	ROS_INFO_STREAM( "cloud filtered along camera z axis. "
			<< "(input cloud size:" << cloud->size() << ") (deep threshold:" << thDeep
			<< ") (closer cloud size:" << cloudCloser->size() << ") (further cloud size:" << cloudFurther->size() << ")");

	// prepare data for output
	res.cloud_closer = PCManager::cloudToRosMsg( cloudCloser);
	res.cloud_further = PCManager::cloudToRosMsg( cloudFurther);
	res.used_deep_threshold = thDeep;

	return true;
}

// Initialize node
int main(int argc, char **argv){
	init(argc, argv, srvm::SRV_NAME_DEEP_FILTER);
	NodeHandle n;

	ServiceServer service = n.advertiseService( srvm::SRV_NAME_DEEP_FILTER, deepFiltering);
	spin();

	return 0;
}
