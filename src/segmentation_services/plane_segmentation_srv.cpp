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

// default params names
static const double PLANE_NORMAL_DISTANCE_WEIGTH = 0.001f; // 0.01f;
static const double PLANE_DISTANCE_TH = 0.007f;
static const int PLANE_MAX_ITERATION_LIMIT = 1000;
static const double PLANE_EPS_ANGLE_TH = 0.0f;
static const double PLANE_MIN_OPENING_ANGLE_DEGREE = 0.0f; // degree
static const double PLANE_MAX_OPENING_ANGLE_DEGREE = 10.0f; // degree

// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool ransacPlaneDetaction( PrimitiveSegmentation::Request  &req, PrimitiveSegmentation::Response &res){

	// get input points
	PCLCloudPtr cloud = PCManager::cloudForRosMsg( req.cloud); 		// input cloud
	PCLNormalPtr normals = PCManager::normForRosMsg( req.normals);	// input norms

	// initialise input parameter
	int maxIterations;
	double normalDistanceWeight, distanceThreshold, minRadiusLimit, maxRadiusLimit, epsAngleTh, minOpeningAngle, maxOpeningAngle;

    // get params or set to default values
    nh_ptr->param(srvm::PARAM_NAME_PLANE_NORMAL_DISTANCE_WEIGHT,
                  normalDistanceWeight, PLANE_NORMAL_DISTANCE_WEIGTH);
    nh_ptr->param(srvm::PARAM_NAME_PLANE_DISTANCE_TH,
                  distanceThreshold, PLANE_DISTANCE_TH);
    nh_ptr->param(srvm::PARAM_NAME_PLANE_MAX_ITERATION_LIMIT,
                  maxIterations, PLANE_MAX_ITERATION_LIMIT);
    nh_ptr->param(srvm::PARAM_NAME_PLANE_EPS_ANGLE_TH,
                  epsAngleTh, PLANE_EPS_ANGLE_TH);
    nh_ptr->param(srvm::PARAM_NAME_PLANE_MIN_OPENING_ANGLE_DEGREE,
                  minOpeningAngle, PLANE_MIN_OPENING_ANGLE_DEGREE);
    nh_ptr->param(srvm::PARAM_NAME_PLANE_MAX_OPENING_ANGLE_DEGREE,
                  maxOpeningAngle, PLANE_MAX_OPENING_ANGLE_DEGREE);

	// apply RANSAC
	SACSegmentationFromNormals< PointXYZ, Normal> seg;
	ModelCoefficients::Ptr coefficients_plane( new ModelCoefficients);
	PointIndices::Ptr inliers_plane( new PointIndices);
	seg.setOptimizeCoefficients( true);
	seg.setModelType( SACMODEL_PLANE);
	seg.setMethodType( SAC_RANSAC);
	seg.setNormalDistanceWeight( normalDistanceWeight); // the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal. (The Euclidean distance will have weight 1-w.)
	seg.setMaxIterations( maxIterations);
	seg.setDistanceThreshold( distanceThreshold);
	seg.setInputCloud( cloud);
	seg.setInputNormals( normals);
	seg.setEpsAngle( epsAngleTh);
	seg.setMinMaxOpeningAngle( minOpeningAngle / 180.0  * M_PI, maxOpeningAngle / 180.0 * M_PI);
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
	ros::init(argc, argv, srvm::SRV_NAME_RANSAC_PLANE_FILTER);
    ros::NodeHandle nh;
    nh_ptr = &nh;


    ros::ServiceServer service = nh.advertiseService( srvm::SRV_NAME_RANSAC_PLANE_FILTER, ransacPlaneDetaction);
	ros::spin();

	return 0;
}
