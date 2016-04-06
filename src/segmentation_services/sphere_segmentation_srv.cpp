#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>				// pcl to ros conversion library
#include <pcl/segmentation/sac_segmentation.h>	// ransac

#include "pitt_msgs/PrimitiveSegmentation.h"	// services and messages

#include "../point_cloud_library/pc_manager.h"	// my static library
#include "../point_cloud_library/srv_manager.h"

using namespace pcm;
using namespace pcl;
using namespace std;
using namespace srvm;
//using namespace pitt_object_table_segmentation;
using namespace pitt_msgs;


// default parameters value (set parameter to be < 0 to use default value)
const float NORMAL_DISTANCE_WEIGHT_DEFAULT = 0.001f; //0.0001f;
const float DISTANCE_THRESHOLD_DEFAULT = 0.007f; // 0.7f;
const int MAX_ITERATION_DEFAULT = 1000;//20;
const float MIN_RADIUS_LIMIT = 0.005;
const float MAX_RADIUS_LIMIT = 0.500;
const float EPS_ANGLE = 0.0f;
const float MIN_OPENING_ANGLE = 100.0f; // degree
const float MAX_OPENING_ANGLE = 180.0f; // degree

// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool ransacSphereDetaction( PrimitiveSegmentation::Request  &req, PrimitiveSegmentation::Response &res){

	// get input points
	PCLCloudPtr cloud = PCManager::cloudForRosMsg( req.cloud); 		// input cloud
	PCLNormalPtr normals = PCManager::normForRosMsg( req.normals);	// input norms

	// initialise input parameter
	int maxIterations;
	double normalDistanceWeight, distanceThreshold, minRadiusLimit, maxRadiusLimit, epsAngleTh, minOpeningAngle, maxOpeningAngle;
	if( req.normal_distance_weight >= 0.0f)
		normalDistanceWeight = req.normal_distance_weight;
	else normalDistanceWeight = NORMAL_DISTANCE_WEIGHT_DEFAULT;
	if( req.distance_threshold >= 0.0f)
		distanceThreshold = req.distance_threshold;
	else distanceThreshold = DISTANCE_THRESHOLD_DEFAULT;
	if( req.max_iterations >= 0)
		maxIterations = req.max_iterations;
	else maxIterations = MAX_ITERATION_DEFAULT;
	if( req.min_radius_limit >= 0.0f)
		minRadiusLimit = req.min_radius_limit;
	else minRadiusLimit = MIN_RADIUS_LIMIT;
	if( req.max_radius_limit >= 0.0f)
		maxRadiusLimit = req.max_radius_limit;
	else maxRadiusLimit = MAX_RADIUS_LIMIT;
	if( req.eps_angle_threshold >= 0.0f)
		epsAngleTh = req.eps_angle_threshold;
	else epsAngleTh = EPS_ANGLE;
	if( req.min_opening_angle_degree >= 0.0f)
		minOpeningAngle = req.min_opening_angle_degree;
	else minOpeningAngle = MIN_OPENING_ANGLE;
	if( req.max_opening_angle_degree >= 0.0f)
		maxOpeningAngle = req.max_opening_angle_degree;
	else maxOpeningAngle = MAX_OPENING_ANGLE;

	// apply RANSAC
	SACSegmentationFromNormals< PointXYZ, Normal> seg;
	ModelCoefficients::Ptr coefficients_sphere( new ModelCoefficients);
	PointIndices::Ptr inliers_sphere( new PointIndices);
	seg.setOptimizeCoefficients( true);
	seg.setModelType( SACMODEL_SPHERE);
	seg.setMethodType( SAC_RANSAC);
	seg.setNormalDistanceWeight( normalDistanceWeight);
	seg.setMaxIterations( maxIterations);
	seg.setDistanceThreshold( distanceThreshold);
	seg.setRadiusLimits( minRadiusLimit, maxRadiusLimit);
	seg.setInputCloud( cloud);
	seg.setInputNormals( normals);
	seg.setEpsAngle( epsAngleTh);
	seg.setMinMaxOpeningAngle( minOpeningAngle / 180.0  * M_PI, maxOpeningAngle / 180.0 * M_PI);
	// Obtain the sphere inliers and coefficients
	seg.segment( *inliers_sphere, *coefficients_sphere);

	// set returning value
	res.inliers = PCManager::inlierToVectorMsg( inliers_sphere);	// inlier w.r.t. the input cloud
	res.coefficients = PCManager::coefficientToVectorMsg( coefficients_sphere);
	// set returning center of mass (= the center of the sphere)
	if( coefficients_sphere->values.size() > 0){
		res.x_centroid = coefficients_sphere->values[ 0];
		res.y_centroid = coefficients_sphere->values[ 1];
		res.z_centroid = coefficients_sphere->values[ 2];
		ROS_INFO(" estimated sphere centeroid: %f  %f  %f", coefficients_sphere->values[ 0], coefficients_sphere->values[ 1], coefficients_sphere->values[ 2]);
	}

	// coeff 0:centreX, 1:centreY, 2:centreZ, 3:radious
//	if( inliers_sphere->indices.size() > 0)
//		cout << " sphere found ... inliers:" << inliers_sphere->indices.size() <<
//				" Cx:" << coefficients_sphere->values[ 0] <<
//				" Cy:" << coefficients_sphere->values[ 1] <<
//				" Cz:" << coefficients_sphere->values[ 2] <<
//				" radius:" << coefficients_sphere->values[ 3] << endl;
//	else cout << " NO sphere found" << endl;

	// set used parameters
	res.used_distance_threshold = distanceThreshold;
	res.used_eps_angle_th = epsAngleTh;
	res.used_max_iterations = maxIterations;
	res.used_max_opening_angle_degree = maxOpeningAngle;
	res.used_max_radius_limit = maxRadiusLimit;
	res.used_min_opening_angle_degree = minOpeningAngle;
	res.used_min_radius_limit = minRadiusLimit;
	res.used_normal_distance_weight = normalDistanceWeight;

	return true;
}


// Initialize node
int main(int argc, char **argv){
	ros::init(argc, argv, srvm::SRV_NAME_RANSAC_SPHERE_FILTER);
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService( srvm::SRV_NAME_RANSAC_SPHERE_FILTER, ransacSphereDetaction);
	ros::spin();

	return 0;
}
