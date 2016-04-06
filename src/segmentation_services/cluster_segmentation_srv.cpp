#include "ros/ros.h"
#include <iostream>
// services and messages
#include "pitt_msgs/ClusterSegmentation.h"
#include "pitt_msgs/InliersCluster.h"
// pcl to ros conversion library
#include <pcl_ros/point_cloud.h>
// clustering
#include <pcl/segmentation/extract_clusters.h>
// for point clod 2 ros msg
#include <std_msgs/String.h>
// for traslating from PointCloud<> to PointCloud2
#include <pcl_conversions/pcl_conversions.h>
// for my point cloud static library


#include "../point_cloud_library/pc_manager.h"
#include "../point_cloud_library/srv_manager.h"


// useful class name used
using namespace pcl;
using namespace std;
using namespace pcm;
using namespace srvm;
using namespace sensor_msgs;
//using namespace pitt_object_table_segmentation;
using namespace pitt_msgs;

// default parameters value (set parameter to be < 0 to use default value) (e.g. minimum distances between different objects)
const float TOLLERANCE_DEFAULT = 0.03f; // distanza (in metri) per considerare due punti appartenenti allo stesso cluster
const float MIN_CLUSTER_RATE_DEFAULT = 0.01f;// (001.0%) rate w.r.t. the total number of points !!!! MIN > MAX !!!!
const float MAX_CLUSTER_RATE_DEFAULT = 0.99f;// (099.0%)
const int MIN_INPUT_SIZE_DEFAULT =   30; // minimum number of points

// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool clusterize(ClusterSegmentation::Request  &req, ClusterSegmentation::Response &res){

	// get data and convert in useful format considering also default
	PCLCloudPtr cloud = pcm::PCManager::cloudForRosMsg( req.cloud);
	double tollerance, minClasterSizeRate, maxClasterSizeRate;
	int minInputSize;
	if( req.cluster_tolerance >= 0.0f)
		tollerance = req.cluster_tolerance;
	else tollerance = TOLLERANCE_DEFAULT;
	if( req.min_cluster_size_rate >= 0.0f)
		minClasterSizeRate = req.min_cluster_size_rate;
	else minClasterSizeRate = MIN_CLUSTER_RATE_DEFAULT;
	if( req.max_cluster_size_rate >= 0.0f)
		maxClasterSizeRate = req.max_cluster_size_rate;
	else maxClasterSizeRate = MAX_CLUSTER_RATE_DEFAULT;
	if( req.min_input_size >= 0.0f)
		minInputSize = req.min_input_size;
	else minInputSize = MIN_INPUT_SIZE_DEFAULT;

	if( cloud->points.size() >= minInputSize){ // skip if input cloud is too small

		// Creating the KdTree object for the search method of the extraction
		search::KdTree< PointXYZ>::Ptr tree (new search::KdTree< PointXYZ>);
		tree->setInputCloud ( cloud);

		// compute clusters
		vector< PointIndices> cluster_indices;
		EuclideanClusterExtraction< PointXYZ> ec;
		ec.setClusterTolerance( tollerance); // in meters
		ec.setMinClusterSize( round(cloud->points.size () * minClasterSizeRate)); // percentage
		ec.setMaxClusterSize( round(cloud->points.size () * maxClasterSizeRate));
		//ROS_ERROR( "%d  %f   %f", cloud->points.size(), cloud->points.size () * minClasterSizeRate, cloud->points.size () * maxClasterSizeRate);
		ec.setSearchMethod( tree);
		ec.setInputCloud( cloud);
		ec.extract( cluster_indices);

		// for all the cluster
		for ( vector< pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
			// build inliers output
			vector< int>* inlier (new std::vector< int>);
			PCLCloudPtr cloudExtraxted (new PCLCloud);
			// build centroid output
			float xSumm = 0, ySumm = 0, zSumm = 0;
			int cnt = 1;
			// for all the point of a cluster (create a new point cloud for every clusters)
			for ( vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
				// set inlier
				inlier->push_back( *pit);
				// set a point of the cloud
				PointXYZ* p ( new PointXYZ( cloud->points[*pit].x, cloud->points[*pit].y, cloud->points[*pit].z));
				cloudExtraxted->push_back( *p);

				// save summ to compute avarage
				xSumm += cloud->points[*pit].x;
				ySumm += cloud->points[*pit].y;
				zSumm += cloud->points[*pit].z;
				cnt++;
			}

			// create returning object
			InliersCluster* cluster ( new InliersCluster);
			cluster->inliers = *inlier;
			cluster->cloud = PCManager::cloudToRosMsg( cloudExtraxted);
			// compute and assign to output the cluster centroid
			cluster->x_centroid = xSumm / cnt;
			cluster->y_centroid = ySumm / cnt;
			cluster->z_centroid = zSumm / cnt;
			// add to returning values
			res.cluster_objs.push_back( *cluster);
		}
	}

	// set used parameters
	res.used_cluster_tolerance = tollerance;
	res.used_max_cluster_size_rate = maxClasterSizeRate;
	res.used_min_cluster_size_rate = minClasterSizeRate;
	res.used_min_input_size = minInputSize;

	return true;
}




// Initialize node
int main(int argc, char **argv){
	ros::init(argc, argv, srvm::SRV_NAME_CUSTER_FILTER);
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService( srvm::SRV_NAME_CUSTER_FILTER, clusterize);
	ros::spin();

	return 0;
}
