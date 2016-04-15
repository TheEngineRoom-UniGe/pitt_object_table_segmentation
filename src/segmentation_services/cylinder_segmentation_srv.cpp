#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>				// pcl to ros conversion library
#include <pcl/segmentation/sac_segmentation.h>	// ransac

#include "pitt_msgs/PrimitiveSegmentation.h"	// services and messages

#include "../point_cloud_library/pc_manager.h"				// my static library
#include "../point_cloud_library/srv_manager.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace pcm;
using namespace pcl;
using namespace srvm;
using namespace std;
using namespace pitt_msgs;

ros::NodeHandle* nh_ptr = NULL;

// default params names
static const double CYLINDER_NORMAL_DISTANCE_WEIGTH = 0.001; //0.0001;
static const double CYLINDER_DISTANCE_TH = 0.008; //0.007;//0.7;
static const double CYLINDER_MIN_RADIUS_LIMIT = 0.005;
static const double CYLINDER_MAX_RADIUS_LIMIT = 0.500;
static const int CYLINDER_MAX_ITERATION_LIMIT = 1000; //20;
static const double CYLINDER_EPS_ANGLE_TH = 0.0001;
static const double CYLINDER_MIN_OPENING_ANGLE_DEGREE = 50.0; // degree
static const double CYLINDER_MAX_OPENING_ANGLE_DEGREE = 180.0; // degree

// vector or point data structure
struct vector3d {
  float x;
  float y;
  float z;
} ;

// visualization variables
const bool VISUALIZE_RESULT = false;
boost::shared_ptr< visualization::PCLVisualizer> vis;	// to visualize cloud
boost::thread vis_thread;
boost::mutex vis_mutex;

void visSpin(){
    vis->spin();
}

// retrieve the direction of the cone axes and normalize it as a versor
vector3d getNormalizeAxesDirectionVector( ModelCoefficients::Ptr coefficients){
	float norm = sqrt( coefficients->values[ 3] * coefficients->values[ 3] +
					coefficients->values[ 4] * coefficients->values[ 4] + coefficients->values[ 5] * coefficients->values[ 5]);
	vector3d direction;
	direction.x = coefficients->values[ 3] / norm;
	direction.y = coefficients->values[ 4] / norm;
	direction.z = coefficients->values[ 5] / norm;
	return direction;
}

// get a point belong to the cone axes (w.r.t to a parameter t which can be wathever ?? !!)
vector3d getPointOnAxes( ModelCoefficients::Ptr coefficients, vector3d direction, float t){
	vector3d point;
	point.x = coefficients->values[ 0] + direction.x * t;
	point.y = coefficients->values[ 1] + direction.y * t;
	point.z = coefficients->values[ 2] + direction.z * t;
	return( point);
}

// get the vector that connect two points
vector3d getVectorBetweenPoints( vector3d p1, vector3d p2){
	vector3d vectorPoints;
	vectorPoints.x = p2.x - p1.x;
	vectorPoints.y = p2.y - p1.y;
	vectorPoints.z = p2.z - p1.z;
	return( vectorPoints);
}

// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool ransacCylinderDetaction( PrimitiveSegmentation::Request  &req, PrimitiveSegmentation::Response &res){

	// get input points
	PCLCloudPtr cloud = PCManager::cloudForRosMsg( req.cloud); 		// input cloud
	PCLNormalPtr normals = PCManager::normForRosMsg( req.normals);	// input norms

	// initialize input parameters
	int maxIterations;
	double normalDistanceWeight, distanceThreshold, minRadiusLimit, maxRadiusLimit, epsAngleTh, minOpeningAngle, maxOpeningAngle;

    // get params or set to default values
    nh_ptr->param(srvm::PARAM_NAME_CYLINDER_NORMAL_DISTANCE_WEIGHT,
                  normalDistanceWeight, CYLINDER_NORMAL_DISTANCE_WEIGTH);
    nh_ptr->param(srvm::PARAM_NAME_CYLINDER_DISTANCE_TH,
                  distanceThreshold, CYLINDER_DISTANCE_TH);
    nh_ptr->param(srvm::PARAM_NAME_CYLINDER_MAX_ITERATION_LIMIT,
                  maxIterations, CYLINDER_MAX_ITERATION_LIMIT);
    nh_ptr->param(srvm::PARAM_NAME_CYLINDER_MIN_RADIUS_LIMIT,
                  minRadiusLimit, CYLINDER_MIN_RADIUS_LIMIT);
    nh_ptr->param(srvm::PARAM_NAME_CYLINDER_MAX_RADIUS_LIMIT,
                  maxRadiusLimit, CYLINDER_MAX_RADIUS_LIMIT);
    nh_ptr->param(srvm::PARAM_NAME_CYLINDER_EPS_ANGLE_TH,
                  epsAngleTh, CYLINDER_EPS_ANGLE_TH);
    nh_ptr->param(srvm::PARAM_NAME_CYLINDER_MIN_OPENING_ANGLE_DEGREE,
                  minOpeningAngle, CYLINDER_MIN_OPENING_ANGLE_DEGREE);
    nh_ptr->param(srvm::PARAM_NAME_CYLINDER_MAX_OPENING_ANGLE_DEGREE,
                  maxOpeningAngle, CYLINDER_MAX_OPENING_ANGLE_DEGREE);
    
	// apply RANSAC
	SACSegmentationFromNormals< PointXYZ, Normal> seg;
	ModelCoefficients::Ptr coefficients_cyliner( new ModelCoefficients);
	PointIndices::Ptr inliers_cylinder( new PointIndices);
	seg.setOptimizeCoefficients( true);
	seg.setModelType( SACMODEL_CYLINDER);
	seg.setMethodType( SAC_RANSAC);
	seg.setNormalDistanceWeight( normalDistanceWeight);
	seg.setMaxIterations( maxIterations);
	seg.setDistanceThreshold( distanceThreshold);
	seg.setRadiusLimits( minRadiusLimit, maxRadiusLimit);
	seg.setInputCloud( cloud);
	seg.setInputNormals( normals);
	seg.setEpsAngle( epsAngleTh);
	seg.setMinMaxOpeningAngle( minOpeningAngle / 180.0  * M_PI, maxOpeningAngle / 180.0 * M_PI);
	// Obtain the plane inliers and coefficients
	seg.segment( *inliers_cylinder, *coefficients_cyliner);


	// compute the center of mass (hp: uniform material)
	vector3d centroid;
	PCLCloudPtr projected_cloud( new PCLCloud);
	float height = -1.0f; // it is the maxim distances between pair of points (of the incoming cloud) projected into the cone axis
	if( inliers_cylinder->indices.size() > 0){
		// normalize direction vector
		vector3d normalizedAxesDirection = getNormalizeAxesDirectionVector( coefficients_cyliner); // [x,y,z]

		// get the cone axes (2points) (with respect to direction and apex)
		vector3d A1 = getPointOnAxes( coefficients_cyliner, normalizedAxesDirection, -1.0f);
		vector3d A2 = getPointOnAxes( coefficients_cyliner, normalizedAxesDirection, +1.0f);

		// project all the points of the cloud on the cone axis        p = A1 + [dot( A1P, A1A2) / dot( A1A2, A1A2)] * A1A2 = A1 + G * A1A2
		//PCLCloudPtr projected_cloud = new PCLCloud( cloud->size());
		vector3d A1A2 = getVectorBetweenPoints( A1, A2);
		float gDivis = A1A2.x * A1A2.x + A1A2.y * A1A2.y + A1A2.z * A1A2.z;
		for( int i = 0; i < cloud->size(); i++){
			vector3d P;
			P.x = cloud->points[ i].x;
			P.y = cloud->points[ i].y;
			P.z = cloud->points[ i].z;
			vector3d A1P = getVectorBetweenPoints( A1, P);
			float G = ( A1P.x * A1A2.x + A1P.y * A1A2.y + A1P.z * A1A2.z) / gDivis;
			PointXYZ *p ( new PointXYZ( A1.x + G * A1A2.x, A1.y + G * A1A2.y, A1.z + G * A1A2.z));
			projected_cloud->push_back( *p);
		}

		// get the two points with the highest relative distance
		int idx1 = -1, idx2 = -1;
		for( int i = 0; i < projected_cloud->size(); i++){
			for( int j = 0; j < projected_cloud->size(); j++){
				if( i > j){
					PointXYZ p1 = projected_cloud->points[ i];
					PointXYZ p2 = projected_cloud->points[ j];
					float distance = sqrt(( p1.x - p2.x) * ( p1.x - p2.x) + ( p1.y - p2.y) * ( p1.y - p2.y) + ( p1.z - p2.z) * ( p1.z - p2.z));
					if( distance > height){
						height = distance; // save the new cone height
						idx1 = i;
						idx2 = j;
					}
				}
			}
		}

		// compute it (1/2h above the base)
		centroid.x = ( projected_cloud->points[ idx1].x + projected_cloud->points[ idx2].x) / 2;
		centroid.y = ( projected_cloud->points[ idx1].y + projected_cloud->points[ idx2].y) / 2;
		centroid.z = ( projected_cloud->points[ idx1].z + projected_cloud->points[ idx2].z) / 2;

		ROS_INFO(" estimated height: %f", height);
		ROS_INFO(" estimated cone centeroid: %f  %f  %f", centroid.x, centroid.y, centroid.z);

		if( VISUALIZE_RESULT){
            boost::mutex::scoped_lock lock(vis_mutex);
			PCManager::updateVisor( vis, cloud, 200, 200, 200, "cylinder"); // show incoming cloud
			PCManager::updateVisor( vis, projected_cloud, 255, 0, 0, "projected"); // show point of the cloud projected on the cone axis
			PCManager::updateVisor( vis, projected_cloud->points[ idx1], 0, 0, 255, "pMax"); // add the maximum point to compute the height (distance)
			PCManager::updateVisor( vis, projected_cloud->points[ idx2], 0, 0, 255, "pMin"); // add the minimum point to compute the height (distance)
			PCManager::updateVisor( vis, PointXYZ( centroid.x, centroid.y, centroid.z), 0, 255, 0, "centroid"); // add the estimated centroid
		}

	}
	ROS_INFO("-----");

	// set returning value
	vector< float> coefficientVector = PCManager::coefficientToVectorMsg( coefficients_cyliner);
	coefficientVector.push_back( height);	// add height to the coefficients
	res.coefficients = coefficientVector;
	res.inliers = PCManager::inlierToVectorMsg( inliers_cylinder);	// inlier w.r.t. the input cloud
	res.x_centroid = centroid.x;
	res.y_centroid = centroid.y;
	res.z_centroid = centroid.z;

	// coeff 0:centreX, 1:centreY, 2:centreZ, 3:radious
//	if( inliers_cylinder->indices.size() > 0)
//		cout << " cylinder found ... inliers:" << inliers_cylinder->indices.size() <<
//				" pointX:" << coefficientVector[ 0] <<
//				" pointY:" << coefficientVector[ 1] <<
//				" pointZ:" << coefficientVector[ 2] <<
//				"  axisX:" << coefficientVector[ 3] <<
//				"  axisY:" << coefficientVector[ 4] <<
//				"  axisZ:" << coefficientVector[ 5] <<
//				" radius:" << coefficientVector[ 6] <<
//				" height:" << coefficientVector[ 7] << endl;
//	else cout << " NO cylinder found" << endl;

	return true;
}


// Initialize node
int main(int argc, char **argv){
	ros::init(argc, argv, srvm::SRV_NAME_RANSAC_CYLINDER_FILTER);
	ros::NodeHandle nh;
    nh_ptr = &nh;

	if( VISUALIZE_RESULT) {
        vis = PCManager::createVisor("CYLINDER shape segmentation");
        vis->setCameraPosition(8.6096e-05, 0.61526, 0.0408496, 0, 0, 1, 0.0230758, -0.841489, -0.539782);
        vis->setCameraFieldOfView(0.8575);
        vis->setCameraClipDistances(0.00433291, 4.33291);
        vis->setPosition(1, 52);
        vis->setSize(960, 540);
        vis_thread = boost::thread(visSpin);
    }

	ros::ServiceServer service = nh.advertiseService( srvm::SRV_NAME_RANSAC_CYLINDER_FILTER, ransacCylinderDetaction);
	//ros::Rate r(20);
	while ( nh.ok()){
		ros::spinOnce();
		//r.sleep();
	}
    vis_thread.join();
	return 0;
}
