#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>				// pcl to ros conversion library
#include <pcl/segmentation/sac_segmentation.h>	// ransac

#include "pitt_msgs/PrimitiveSegmentation.h"	// services and messages

#include "../point_cloud_library/pc_manager.h"	// my static library
#include "../point_cloud_library/srv_manager.h"

using namespace pcm;
using namespace pcl;
using namespace srvm;
//using namespace pitt_object_table_segmentation;
using namespace pitt_msgs;

ros::NodeHandle* nh_ptr = NULL;


// PLANE SEGMENTATION SERVICE
//sevice name
const string SRV_NAME_RANSAC_PLANE_FILTER = "plane_segmentation_srv";
//ros parameter name
const string NAME_PARAM_PLANE_SRV_NORMAL_DISTANCE_WEIGHT = "/pitt/srv/plane_segmentation/normal_distance_weight";
const string NAME_PARAM_PLANE_SRV_DISTANCE_THRESHOLD = "/pitt/srv/plane_segmentation/distance_th";
const string NAME_PARAM_PLANE_SRV_MAX_ITERATION = "/pitt/srv/plane_segmentation/max_iter_limit";



// default params names
static const double DEFAULT_PARAM_PLANE_SRV_NORMAL_DISTANCE_WEIGHT = 0.001; // 0.01f;
static const double DEFAULT_PARAM_PLANE_SRV_DISTANCE_THRESHOLD = 0.007;
static const int DEFAULT_PARAM_PLANE_SRV_MAX_ITERATION = 1000;


//TODO DELETE
//static const double DEFAULT_PARAM_PLANE_SRV_EPS_ANGLE_TH = 0.0;
//static const double DEFAULT_PARAM_PLANE_SRV_MIN_OPENING_ANGLE = 0.0; // degree
//static const double DEFAULT_PARAM_PLANE_SRV_MAX_OPENING_ANGLE = 10.0; // degree
//const string NAME_PARAM_PLANE_SRV_EPS_ANGLE_TH = "/pitt/srv/plane_segmentation/eps_angle_th";
//const string NAME_PARAM_PLANE_SRV_MIN_OPENING_ANGLE = "/pitt/srv/plane_segmentation/min_opening_angle_deg";
//const string NAME_PARAM_PLANE_SRV_MAX_OPENING_ANGLE = "/pitt/srv/plane_segmentation/max_opening_angle_deg";
//nh_ptr->param(NAME_PARAM_PLANE_SRV_EPS_ANGLE_TH,
//		inputPlaneEpsAngleTh, DEFAULT_PARAM_PLANE_SRV_EPS_ANGLE_TH);
//nh_ptr->param(NAME_PARAM_PLANE_SRV_MIN_OPENING_ANGLE,
//		inputPlaneMinOpeningAngle, DEFAULT_PARAM_PLANE_SRV_MIN_OPENING_ANGLE);
//nh_ptr->param(NAME_PARAM_PLANE_SRV_MAX_OPENING_ANGLE,
//		inputPlaneMaxOpeningAngle, DEFAULT_PARAM_PLANE_SRV_MAX_OPENING_ANGLE);
//seg.setEpsAngle( inputPlaneEpsAngleTh);
//seg.setMinMaxOpeningAngle( inputPlaneMinOpeningAngle / 180.0  * M_PI, inputPlaneMaxOpeningAngle / 180.0 * M_PI);


// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool ransacPlaneDetaction( PrimitiveSegmentation::Request  &req, PrimitiveSegmentation::Response &res){

	// get input points
	PCLCloudPtr cloud = PCManager::cloudForRosMsg( req.cloud); 		// input cloud
	PCLNormalPtr normals = PCManager::normForRosMsg( req.normals);	// input norms

	// initialise input parameter
	int inputPlaneMaxIteration;
	double inputPlaneNormalDistanceWeight, inputPlaneDistanceThreshold, minRadiusLimit, maxRadiusLimit, inputPlaneEpsAngleTh, inputPlaneMinOpeningAngle, inputPlaneMaxOpeningAngle;

    // get params or set to default values
    nh_ptr->param(NAME_PARAM_PLANE_SRV_NORMAL_DISTANCE_WEIGHT,
                  inputPlaneNormalDistanceWeight, DEFAULT_PARAM_PLANE_SRV_NORMAL_DISTANCE_WEIGHT);
    nh_ptr->param(NAME_PARAM_PLANE_SRV_DISTANCE_THRESHOLD,
                  inputPlaneDistanceThreshold, DEFAULT_PARAM_PLANE_SRV_DISTANCE_THRESHOLD);
    nh_ptr->param(NAME_PARAM_PLANE_SRV_MAX_ITERATION,
                  inputPlaneMaxIteration, DEFAULT_PARAM_PLANE_SRV_MAX_ITERATION);


	// apply RANSAC
	SACSegmentationFromNormals< PointXYZRGB, Normal> seg;
	ModelCoefficients::Ptr coefficients_plane( new ModelCoefficients);
	PointIndices::Ptr inliers_plane( new PointIndices);
	seg.setOptimizeCoefficients( true);
	seg.setModelType( SACMODEL_PLANE);
	seg.setMethodType( SAC_RANSAC);
	seg.setNormalDistanceWeight( inputPlaneNormalDistanceWeight); // the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal. (The Euclidean distance will have weight 1-w.)
	seg.setMaxIterations( inputPlaneMaxIteration);
	seg.setDistanceThreshold( inputPlaneDistanceThreshold);
	seg.setInputCloud( cloud);
	seg.setInputNormals( normals);

	// minRadiousLimit e maxRadiousLimit are not used in plane detection
	// Obtain the plane inliers and coefficients
	seg.segment( *inliers_plane, *coefficients_plane);

	// set returning value
	res.inliers = PCManager::inlierToVectorMsg( inliers_plane);	// inlier w.r.t. the input cloud
	res.coefficients = PCManager::coefficientToVectorMsg( coefficients_plane);

	return true;
}


// Initialize node
int main(int argc, char **argv){
	ros::init(argc, argv, SRV_NAME_RANSAC_PLANE_FILTER);
    ros::NodeHandle nh;
    nh_ptr = &nh;


    ros::ServiceServer service = nh.advertiseService( SRV_NAME_RANSAC_PLANE_FILTER, ransacPlaneDetaction);
	ros::spin();

	return 0;
}
