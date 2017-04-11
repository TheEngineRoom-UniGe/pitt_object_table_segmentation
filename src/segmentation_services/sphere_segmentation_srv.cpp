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
using namespace pitt_msgs;

ros::NodeHandle* nh_ptr = NULL;
//service name
const string SRV_NAME_RANSAC_SPHERE_FILTER = "sphere_segmentation_srv";
//ros parameter name
//SPHERE SEGMENTATION SERVICE
const string NAME_PARAM_SPHERE_SRV_NORMAL_DISTANCE_WEIGHT = "/pitt/srv/sphere_segmentation/normal_distance_weight";
const string NAME_PARAM_SPHERE_SRV_DISTANCE_THRESHOLD = "/pitt/srv/sphere_segmentation/distance_th";
const string NAME_PARAM_SPHERE_SRV_MAX_ITERATION = "/pitt/srv/sphere_segmentation/max_iter_limit";
const string NAME_PARAM_SPHERE_SRV_MIN_RADIUS_LIMIT =  "/pitt/srv/sphere_segmentation/min_radius_limit";
const string NAME_PARAM_SPHERE_SRV_MAX_RADIUS_LIMIT = "/pitt/srv/sphere_segmentation/max_radius_limit";


// default param names
static const double DEFAULT_PARAM_SPHERE_SRV_NORMAL_DISTANCE_WEIGTH = 0.001; //0.0001;
static const double DEFAULT_PARAM_SPHERE_SRV_DISTANCE_THRESHOL = 0.007; // 0.7;
static const double DEFAULT_PARAM_SPHERE_SRV_MIN_RADIUS_LIMIT = 0.005;
static const double DEFAULT_PARAM_SPHERE_SRV_MAX_RADIUS_LIMIT = 0.500;
static const int DEFAULT_PARAM_SPHERE_SRV_MAX_ITERATION = 1000; //20;

//TODO : DELATE
//static const double DEFAULT_PARAM_SPHERE_SRV_EPS_ANGLE_TH = 0.0;
//static const double DEFAULT_PARAM_SPHERE_SRV_MIN_OPENING_ANGLE = 100.0; // degree
//static const double DEFAULT_PARAM_SPHERE_SRV_MAX_OPENING_ANGLE = 180.0; // degree
//const string NAME_PARAM_SPHERE_SRV_EPS_ANGLE_TH = "/pitt/srv/sphere_segmentation/eps_angle_th";
//const string NAME_PARAM_SPHERE_SRV_MIN_OPENING_ANGLE = "/pitt/srv/sphere_segmentation/min_opening_angle_deg";
//const string NAME_PARAM_SPHERE_SRV_MAX_OPENING_ANGLE = "/pitt/srv/sphere_segmentation/max_opening_angle_deg";
//nh_ptr->param(NAME_PARAM_SPHERE_SRV_EPS_ANGLE_TH,
//			  inputSphereSrvEpsAngleTh, DEFAULT_PARAM_SPHERE_SRV_EPS_ANGLE_TH);
//nh_ptr->param(NAME_PARAM_SPHERE_SRV_MIN_OPENING_ANGLE,
//              inputSphereSrvMinOpeningAngle, DEFAULT_PARAM_SPHERE_SRV_MIN_OPENING_ANGLE);
//nh_ptr->param(NAME_PARAM_SPHERE_SRV_MAX_OPENING_ANGLE,
//              inputSphereSrvMaxOpeningAngle, DEFAULT_PARAM_SPHERE_SRV_MAX_OPENING_ANGLE);
//seg.setEpsAngle( inputSphereSrvEpsAngleTh);
//seg.setMinMaxOpeningAngle( inputSphereSrvMinOpeningAngle / 180.0  * M_PI, inputSphereSrvMaxOpeningAngle / 180.0 * M_PI);

// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool ransacSphereDetection(PrimitiveSegmentation::Request &req, PrimitiveSegmentation::Response &res){

	// get input points
	PCLCloudPtr cloud = PCManager::cloudForRosMsg( req.cloud); 		// input cloud
	PCLNormalPtr normals = PCManager::normForRosMsg( req.normals);	// input norms

    // initialize input parameters
	int inputSphereSrvMaxIteration;
	double inputSphereSrvNormalDistanceWeight, inputSphereSrvDistanceThreshold, inputSphereSrvMinRadiusLimit, inputSphereSrvMaxRadiusLimit, inputSphereSrvEpsAngleTh, inputSphereSrvMinOpeningAngle, inputSphereSrvMaxOpeningAngle;

	// get params or set to default values
	nh_ptr->param(NAME_PARAM_SPHERE_SRV_NORMAL_DISTANCE_WEIGHT,
				  inputSphereSrvNormalDistanceWeight, DEFAULT_PARAM_SPHERE_SRV_NORMAL_DISTANCE_WEIGTH);
	nh_ptr->param(NAME_PARAM_SPHERE_SRV_DISTANCE_THRESHOLD,
				  inputSphereSrvDistanceThreshold, DEFAULT_PARAM_SPHERE_SRV_DISTANCE_THRESHOL);
	nh_ptr->param(NAME_PARAM_SPHERE_SRV_MAX_ITERATION,
                  inputSphereSrvMaxIteration, DEFAULT_PARAM_SPHERE_SRV_MAX_ITERATION);
	nh_ptr->param(NAME_PARAM_SPHERE_SRV_MIN_RADIUS_LIMIT,
				  inputSphereSrvMinRadiusLimit, DEFAULT_PARAM_SPHERE_SRV_MIN_RADIUS_LIMIT);
	nh_ptr->param(NAME_PARAM_SPHERE_SRV_MAX_RADIUS_LIMIT,
				  inputSphereSrvMaxRadiusLimit, DEFAULT_PARAM_SPHERE_SRV_MAX_RADIUS_LIMIT);


	// apply RANSAC
	SACSegmentationFromNormals< PointXYZRGB, Normal> seg;
	ModelCoefficients::Ptr coefficients_sphere( new ModelCoefficients);
	PointIndices::Ptr inliers_sphere( new PointIndices);
	seg.setOptimizeCoefficients( true);
	seg.setModelType( SACMODEL_SPHERE);
	seg.setMethodType( SAC_RANSAC);
	seg.setNormalDistanceWeight( inputSphereSrvNormalDistanceWeight);
	seg.setMaxIterations( inputSphereSrvMaxIteration);
	seg.setDistanceThreshold( inputSphereSrvDistanceThreshold);
	seg.setRadiusLimits( inputSphereSrvMinRadiusLimit, inputSphereSrvMaxRadiusLimit);
	seg.setInputCloud( cloud);
	seg.setInputNormals( normals);

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
		ROS_INFO(" estimated sphere centroid: %f  %f  %f", coefficients_sphere->values[ 0], coefficients_sphere->values[ 1], coefficients_sphere->values[ 2]);
	}

	// coeff 0:centreX, 1:centreY, 2:centreZ, 3:radious
//	if( inliers_sphere->indices.size() > 0)
//		cout << " sphere found ... inliers:" << inliers_sphere->indices.size() <<
//				" Cx:" << coefficients_sphere->values[ 0] <<
//				" Cy:" << coefficients_sphere->values[ 1] <<
//				" Cz:" << coefficients_sphere->values[ 2] <<
//				" radius:" << coefficients_sphere->values[ 3] << endl;
//	else cout << " NO sphere found" << endl;

	return true;
}


// Initialize node
int main(int argc, char **argv){
	ros::init(argc, argv, SRV_NAME_RANSAC_SPHERE_FILTER);
	ros::NodeHandle nh;
	nh_ptr = &nh;

	ros::ServiceServer service = nh.advertiseService(SRV_NAME_RANSAC_SPHERE_FILTER, ransacSphereDetection);
	ros::spin();

	return 0;
}
