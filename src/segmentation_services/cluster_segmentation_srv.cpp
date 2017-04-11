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
#include <pcl/impl/point_types.hpp>

#include "../point_cloud_library/pc_manager.h"
#include "../point_cloud_library/srv_manager.h"


// useful class name used
using namespace pcl;
using namespace std;
using namespace pcm;
using namespace srvm;
using namespace sensor_msgs;
using namespace pitt_msgs;

ros::NodeHandle* nh_ptr = NULL;
//ros parameter names
//CLUSTER SEGMENTATION SERVICE
const string NAME_PARAM_CLUSTER_SRV_TOLERANCE = "/pitt/srv/cluster_segmentation/tolerance";
const string NAME_PARAM_CLUSTER_SRV_MIN_SIZE_RATE = "/pitt/srv/cluster_segmentation/min_rate";
const string NAME_PARAM_CLUSTER_SRV_MAX_SIZE_RATE = "/pitt/srv/cluster_segmentation/max_rate";
const string NAME_PARAM_CLUSTER_SRV_MIN_INPUT_SIZE = "/pitt/srv/cluster_segmentation/min_input_size";
// default parameters value (set parameter to be < 0 to use default value) (e.g. minimum distances between different objects)
const double DEFAULT_CLUSTER_SRV_TOLERANCE = 0.03; // distanza (in metri) per considerare due punti appartenenti allo stesso cluster
const double DEFAULT_PARAM_CLUSTER_SRV_MIN_SIZE_RATE = 0.01;// (001.0%) rate w.r.t. the total number of points !!!! MIN > MAX !!!!
const double DEFAULT_PARAM_CLUSTER_SRV_MAX_SIZE_RATE = 0.99;// (099.0%)
const int DEFAULT_PARAM_CLUSTER_SRV_MIN_INPUT_SIZE =   30; // minimum number of points

// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool clusterize(ClusterSegmentation::Request  &req, ClusterSegmentation::Response &res){

	// get data and convert in useful format considering also default
	PCLCloudPtr cloud = pcm::PCManager::cloudForRosMsg( req.cloud);
	double inputClusterTolerance, inputClusterMinSizeRate, inputClusterMaxSizeRate;
	int inputClusterMinInputSize;

    nh_ptr->param(NAME_PARAM_CLUSTER_SRV_TOLERANCE,
                  inputClusterTolerance, DEFAULT_CLUSTER_SRV_TOLERANCE);
    nh_ptr->param(NAME_PARAM_CLUSTER_SRV_MIN_SIZE_RATE,
                  inputClusterMinSizeRate, DEFAULT_PARAM_CLUSTER_SRV_MIN_SIZE_RATE);
    nh_ptr->param(NAME_PARAM_CLUSTER_SRV_MAX_SIZE_RATE,
                  inputClusterMaxSizeRate, DEFAULT_PARAM_CLUSTER_SRV_MAX_SIZE_RATE);
    nh_ptr->param(NAME_PARAM_CLUSTER_SRV_MIN_INPUT_SIZE,
                  inputClusterMinInputSize, DEFAULT_PARAM_CLUSTER_SRV_MIN_INPUT_SIZE);

	if( cloud->points.size() >= inputClusterMinInputSize){ // skip if input cloud is too small

		// Creating the KdTree object for the search method of the extraction
		search::KdTree< PointXYZRGB>::Ptr tree (new search::KdTree< PointXYZRGB>);
		tree->setInputCloud ( cloud);

		// compute clusters
		vector< PointIndices> cluster_indices;
		EuclideanClusterExtraction< PointXYZRGB> ec;
		ec.setClusterTolerance(inputClusterTolerance); // in meters
		ec.setMinClusterSize( round(cloud->points.size () * inputClusterMinSizeRate)); // percentage
		ec.setMaxClusterSize( round(cloud->points.size () * inputClusterMaxSizeRate));
		//ROS_ERROR( "%d  %f   %f", cloud->points.size(), cloud->points.size () * inputClusterMinSizeRate, cloud->points.size () * inputClusterMaxSizeRate);
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
				Eigen::Vector3i rgb_data=cloud->points[*pit].getRGBVector3i();
				PointXYZRGB* p ( new PointXYZRGB(rgb_data(0),rgb_data(1),rgb_data(2)));
				p->x=cloud->points[*pit].x;
				p->y=cloud->points[*pit].y;
				p->z=cloud->points[*pit].z;
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

	return true;
}




// Initialize node
int main(int argc, char **argv){
	ros::init(argc, argv, srvm::SRV_NAME_CUSTER_FILTER);
    ros::NodeHandle nh;
    nh_ptr = &nh;

	ros::ServiceServer service = nh.advertiseService( srvm::SRV_NAME_CUSTER_FILTER, clusterize);
	ros::spin();

	return 0;
}
