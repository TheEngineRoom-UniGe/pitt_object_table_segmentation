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

// default param names
static const double SPHERE_NORMAL_DISTANCE_WEIGTH = 0.001; //0.0001;
static const double SPHERE_DISTANCE_TH = 0.007; // 0.7;
static const double SPHERE_MIN_RADIUS_LIMIT = 0.005;
static const double SPHERE_MAX_RADIUS_LIMIT = 0.500;
static const int SPHERE_MAX_ITERATION_LIMIT = 1000; //20;
static const double SPHERE_EPS_ANGLE_TH = 0.0;
static const double SPHERE_MIN_OPENING_ANGLE_DEGREE = 100.0; // degree
static const double SPHERE_MAX_OPENING_ANGLE_DEGREE = 180.0; // degree

// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool ransacSphereDetection(PrimitiveSegmentation::Request &req, PrimitiveSegmentation::Response &res){

	// get input points
	PCLCloudPtr cloud = PCManager::cloudForRosMsg( req.cloud); 		// input cloud
	PCLNormalPtr normals = PCManager::normForRosMsg( req.normals);	// input norms

    // initialize input parameters
	int maxIterations;
	double normalDistanceWeight, distanceThreshold, minRadiusLimit, maxRadiusLimit, epsAngleTh, minOpeningAngle, maxOpeningAngle;

	// get params or set to default values
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_NORMAL_DISTANCE_WEIGHT,
				  normalDistanceWeight, SPHERE_NORMAL_DISTANCE_WEIGTH);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_DISTANCE_TH,
				  distanceThreshold, SPHERE_DISTANCE_TH);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MAX_ITERATION_LIMIT,
                  maxIterations, SPHERE_MAX_ITERATION_LIMIT);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MIN_RADIUS_LIMIT,
				  minRadiusLimit, SPHERE_MIN_RADIUS_LIMIT);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MAX_RADIUS_LIMIT,
				  maxRadiusLimit, SPHERE_MAX_RADIUS_LIMIT);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_EPS_ANGLE_TH,
				  epsAngleTh, SPHERE_EPS_ANGLE_TH);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MIN_OPENING_ANGLE_DEGREE,
                  minOpeningAngle, SPHERE_MIN_OPENING_ANGLE_DEGREE);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MAX_OPENING_ANGLE_DEGREE,
                  maxOpeningAngle, SPHERE_MAX_OPENING_ANGLE_DEGREE);

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

	return true;
}


// Initialize node
int main(int argc, char **argv){
	ros::init(argc, argv, srvm::SRV_NAME_RANSAC_SPHERE_FILTER);
	ros::NodeHandle nh;
	nh_ptr = &nh;

	ros::ServiceServer service = nh.advertiseService(srvm::SRV_NAME_RANSAC_SPHERE_FILTER, ransacSphereDetection);
	ros::spin();

	return 0;
}
