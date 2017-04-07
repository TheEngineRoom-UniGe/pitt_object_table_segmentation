#include "ros/ros.h"
#include <iostream>
#include "math.h"								// for spqr function
#include <pcl_ros/point_cloud.h>				// pcl to ros conversion library
#include <std_msgs/String.h>					// for point clod 2 ros msg
#include <pcl/segmentation/sac_segmentation.h>	// for pcl segmentation object
#include <pcl/filters/extract_indices.h>		// for pcl extract a sub cloud from cloud

// services and messages
#include "pitt_msgs/SupportSegmentation.h"
#include "pitt_msgs/Support.h"

// custom class
#include "../point_cloud_library/pc_manager.h"
#include "../point_cloud_library/srv_manager.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// used name space
using namespace pitt_msgs;
using namespace ros;
using namespace pcm;
using namespace pcl;
using namespace srvm;
using namespace boost;
using namespace std;

// used abbreviations (typedef) (PCLCloud, PCLCloudPtr, PCLNormal and PCLNormalPtr) given from PCManager in turn given from PCPrimitive.

// input parameters (or defaults) as global variables
float minIterativeCloudPercentage, minPlanePercentageSize, minVarianceThForHorizontal,
        maxVarianceThForHorizontal, ransacThDistancePointShape, ransacNormalDistanceWeigth;
vector<float> horizontalAxis, supportEdgeRemoveOffset; // TODO check if working
int ransacMaxIteration;

// common global variables
PCLVisualizer vis;
PCLCloudPtr originalCloud;
PCLNormalPtr originalNorms;
boost::thread vis_thread;
boost::mutex vis_mutex;

//TODO: visualization
//
//void visSpin(){
//    vis->spin();
//}

// visualize inlier idx w.r.t to original map for debugging
/*void visualizeInlier( PrimitiveIdxPtr indices){
	vector< PCLCloudPtr> idxCloud = PCManager::getCloudFromIdx( originalCloud, indices);
	for( int i = 0; i < idxCloud.size(); i++)
		PCManager::updateVisor( vis, idxCloud[ i], "idxMap_" + ::to_string( i));
}*/

// set input parameter (global variables) (default if <0 or array are of not 3d structures)
void initializeInputParameters( SupportSegmentation::Request  &req){
    // minimum percentage of number of points of the input cloud and found plane to stop iteratively support searching ([0,1])
    minIterativeCloudPercentage = srvm::getServiceFloatParameter(
            req.min_iterative_cloud_percentual_size, srvm::DEFAULT_PARAM_SRV_SUPPORT_MIN_ITERATIVE_CLOUD_PERCENTAGE);
    minPlanePercentageSize = srvm::getServiceFloatParameter(
            req.min_iterative_plane_percentual_size, srvm::DEFAULT_PARAM_SRV_SUPPORT_MIN_ITERATIVE_SUPPORT_PERCENTAGE);
    // the value (cross product -> 0) to discriminate if a vector is parallel to the ground given in the support service
    maxVarianceThForHorizontal = srvm::getServiceFloatParameter(
            req.variance_threshold_for_horizontal,srvm::DEFAULT_PARAM_SRV_SUPPORT_HORIZONTAL_VARIANCE_THRESHOLD);
    minVarianceThForHorizontal = -1 * maxVarianceThForHorizontal;
    // meters (distance between point to belong to the same shape)
    ransacThDistancePointShape = srvm::getServiceFloatParameter(
            req.ransac_distance_point_in_shape_threshold, srvm::DEFAULT_PARAM_SRV_SUPPORT_RANSAC_IN_SHAPE_DISTANCE_POINT_THRESHOLD);
    ransacNormalDistanceWeigth = srvm::getServiceFloatParameter(
            req.ransac_model_normal_distance_weigth, srvm::DEFAULT_PARAM_SRV_SUPPORT_RANSAC_MODEL_NORMAL_DISTANCE_WEIGHT);
    // max number of iterations for ransac plane segmentation on support detection iterative algorithm
    ransacMaxIteration = srvm::getServiceIntParameter(
            req.ransac_max_iteration_threshold,srvm::DEFAULT_PARAM_SRV_SUPPORT_RANSAC_MAX_ITERATION_THRESHOLD);
    horizontalAxis = srvm::getService3DArrayParameter(req.horizontal_axis,srvm::DEFAULT_PARAM_SRV_SUPPORT_HORIZZONTAL_AXIS);
    supportEdgeRemoveOffset = srvm::getService3DArrayParameter( req.support_edge_remove_offset, srvm::DEFAULT_PARAM_SRV_SUPPORT_EDGE_REMOVE_OFFSET);
}

// use ransac pcl implementation to find the biggest horizontal plane on the scene
SACSegmentationFromNormals< PointXYZRGB, Normal> seg;
void ransacPlaneSegmentator( PCLCloudPtr inputCloud, PCLNormalPtr normals, PointIndices::Ptr &inlierOutput, ModelCoefficients::Ptr &coefficientOutput){
    // Create the segmentation object for the PLANAR model and set all the parameters
    seg.setOptimizeCoefficients( true);
    seg.setModelType( pcl::SACMODEL_PLANE);//::SACMODEL_PARALLEL_PLANE);
    // axes in which compute parallelism or horizontality
    //seg.setAxis( Eigen::Vector3f(0,0,1)); // PARALLEL to z-axis (of the camera)
    // threshold of error between axis (for parallelism or horizontality)
    //seg.setEpsAngle( 0.008); // error in radiants w.r.t. perfect parallelism
    seg.setMethodType( pcl::SAC_RANSAC);
    // the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between
    // point normals and the plane normal. (The Euclidean distance will have weight 1-w.)
    seg.setNormalDistanceWeight(ransacNormalDistanceWeigth);
    seg.setMaxIterations( ransacMaxIteration);
    // max distance between point to the same primitive
    // if smaller => more separate points as shapes
    // if bigger => more surphase as a shape
    seg.setDistanceThreshold( ransacThDistancePointShape);
    seg.setInputCloud( inputCloud);
    seg.setInputNormals( normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inlierOutput, *coefficientOutput);
}

// remove points for a cloud and compute the new cloud without such point
ExtractIndices< PointXYZRGB> extract ( true);
void removePlaneInliner( PCLCloudPtr inputCloud, PointIndices::Ptr &removeIndex, PCLCloudPtr output){
    // compute output: part of the cloud with only index
    extract.setInputCloud( inputCloud);
    extract.setIndices( removeIndex);
    extract.setNegative( false);
    extract.filter( *output);

    // modify input cloud removing the index
    extract.setInputCloud( inputCloud);
    extract.setIndices( removeIndex);
    extract.setNegative( true);
    extract.filter( *inputCloud);
}


// returns true if the array contains the given value, flase otherwise ( used for populateIdxMap() )
bool valueBelongsToArray( int value, PointIndices::Ptr inliers){
    for( int i = 0; i < inliers->indices.size() ; i++)
        if( value == inliers->indices[i])
            return true;
    return false;
}

// given the previous inlier map it create the new inlier idx map
PrimitiveIdxPtr createNewIdxMap( PrimitiveIdxPtr previousInliersMap, PointIndices::Ptr inliers, int level){
    int cnt = 0;
    long nPoint = previousInliersMap->size();
    PrimitiveIdxPtr newInliersMap( new PrimitiveIdx( nPoint));

    for( int p = 0; p < nPoint; p++){
        if( (* previousInliersMap)[ p] > level && (* previousInliersMap)[ p] < 0){
            // propagate the data since this sub cloud does not belongs to the new points
            ( * newInliersMap)[ p] = (* previousInliersMap)[ p];
        } else if ( valueBelongsToArray( (* previousInliersMap)[ p], inliers)){
            // add level tag to sign this point to belong to the new cloud
            ( * newInliersMap)[ p] = level; // negative numbers
        } else{
            // keep counting since the point does not belong to any shapes
            ( * newInliersMap)[ p] = cnt++;
        }
    }
    return( newInliersMap);
}

// returns true if the plane is horizontal, false otherwise
// a plane is horizontal to another if the cross product w.r.t. the normals tends to zero
bool isHorizontalPlane( PCLNormalPtr normal, ModelCoefficients::Ptr coefficients, vector< float> referimentAxis){ // vect<3>(X,Y,Z) for the reference normal to the plane
    // compute normal from cooeficient    nx = a / sqrt( a^2 + b^2 + c^2)  ny = b / sqrt( a^2 + b^2 + c^2)    nz = c / sqrt( a^2 + b^2 + c^2)
    float div = sqrtf( coefficients->values[ 0] * coefficients->values[ 0] + coefficients->values[ 1] * coefficients->values[ 1] + coefficients->values[ 2] * coefficients->values[ 2] );
    float normX = coefficients->values[ 0] / div;
    float normY = coefficients->values[ 1] / div;
    float normZ = coefficients->values[ 2] / div;

    // compute cross product to see if the plane is parallel to the given axes
    float crossX = normY * referimentAxis[ 2] - normZ * referimentAxis[ 1];
    float crossY = normZ * referimentAxis[ 0] - normX * referimentAxis[ 2];
    float crossZ = normX * referimentAxis[ 1] - normY * referimentAxis[ 0];

    // check id the cross product -> 0
    if( 	(( crossX > minVarianceThForHorizontal)	&& ( crossX < maxVarianceThForHorizontal)) &&
           (( crossY > minVarianceThForHorizontal)	&& ( crossY < maxVarianceThForHorizontal)) &&
           (( crossZ > minVarianceThForHorizontal)	&& ( crossZ < maxVarianceThForHorizontal))    )
        return true;
    else return false;
}

// return all the points that, if projected will follow in the given plane.
// this method is designed with respect to the body frame. Particularly, the support are considered to have the z-axis horizontal.

const double inf = std::numeric_limits<double>::infinity();
const double neg_inf = -std::numeric_limits<double>::infinity();

PCLCloudPtr getPointOnPlane( PCLCloudPtr plane, PrimitiveIdxPtr inlierIdx, int mapLevel){
    // get the margin of the plane (x,y) and compute the avarage high (z)
    double xMax = neg_inf, yMax = neg_inf, zMed = 0;
    double xMin = inf, yMin = inf;
    for( int i = 0; i < plane->size(); i++){
        if( plane->points[ i].x > xMax)
            xMax = plane->points[ i].x;
        else if ( plane->points[ i].x < xMin)
            xMin = plane->points[ i].x;

        if( plane->points[ i].y > yMax)
            yMax = plane->points[ i].y;
        else if ( plane->points[ i].y < yMin)
            yMin = plane->points[ i].y;

        // min or max are wrong pheraps it is possible to do it with the mean
        //if( plane->points[ i].z > zMax)
        //	zMax = plane->points[ i].z;
        //else if ( plane->points[ i].z < zMin)
        //	zMin = plane->points[ i].z;
        zMed += plane->points[ i].z;
    }

    // remove some space on the edge of the plane (to do not consider object (e.g. wall) attached to table edge as their are placed on it)
    xMax -= supportEdgeRemoveOffset[ 0];
    xMin += supportEdgeRemoveOffset[ 0];
    yMax -= supportEdgeRemoveOffset[ 1];
    yMin += supportEdgeRemoveOffset[ 1];
    //zMax -= supportEdgeRemoveOffset;
    //zMin += supportEdgeRemoveOffset;
    zMed = zMed / plane->size() + supportEdgeRemoveOffset[ 2];


    PCLCloudPtr out ( new PCLCloud);

    // remove known plane
    PointIndices::Ptr removingIdx( new PointIndices);
    for( int i = 0; i < inlierIdx->size(); i++)
        if( (* inlierIdx)[ i] == mapLevel)
            removingIdx->indices.push_back( i);

    // found the points that are on top of the table
    for( int i = 0; i < originalCloud->size(); i++){
        if( ! valueBelongsToArray( i, removingIdx))
        if( originalCloud->points[ i].x > xMin && originalCloud->points[ i].x < xMax &&
            originalCloud->points[ i].z > zMed &&// originalCloud->points[ i].z < zMax &&
            originalCloud->points[ i].y > yMin && originalCloud->points[ i].y < yMax)
            out->push_back( originalCloud->points[ i]);
    }

    return out;
}

// find horizontal plane iteratively with respect to original cloud (this  is the core service function)
bool findSupports( SupportSegmentation::Request& req, SupportSegmentation::Response& res){
    // initialise parameter from message or from default values
    initializeInputParameters( req);

    // get actual service inputs
    originalCloud = PCManager::cloudForRosMsg( req.input_cloud);
    originalNorms = PCManager::normForRosMsg( req.input_norm);

    // useful variable for iterative horizontal plane segmentation through RANSAC
    PCLCloudPtr iterativeCloud( new PCLCloud( *originalCloud));
    PCLNormalPtr iterativeNorm( new PCLNormal( *originalNorms));
    PointIndicesPtr inliersPlane( new PointIndices);
    ModelCoefficients::Ptr coefficientsPlane( new ModelCoefficients);

    // create beginning inlier structure (original cloud index)
    PrimitiveIdx originalInlierIdx ( originalCloud->size());
    for(int i = 0; i < originalInlierIdx.size(); i++)
        originalInlierIdx[ i] = i;

    // iteratively look for supporting planes and manage the inlier index map
    bool loop = true;
    int idxMapLayer = -2; // -1 is for all the non horizontal planes points
    int cnt = 0;
    PrimitiveIdxPtr newInliersIdx;
    while( loop){
        // look for horizontal plane
        ransacPlaneSegmentator( iterativeCloud, iterativeNorm, inliersPlane, coefficientsPlane);

        // check if stop looping condition are reached
        if( inliersPlane->indices.empty ()){
            loop = false;
            ROS_INFO_STREAM( "support segmentation service exit since no more supports are found.");
            break;
        } else if( iterativeCloud->points.size() < originalCloud->points.size() * minIterativeCloudPercentage){
            loop = false;
            ROS_INFO_STREAM( "support segmentation service exit since too few points are still to be considered.");
            break;
        } else if( inliersPlane->indices.size() < originalCloud->points.size() * minPlanePercentageSize){
            loop = false;
            ROS_INFO_STREAM( "support segmentation service exit since too small support found.");
            break;
        } else { // continue on iterative looping

            // compute the new inlier idx w.r.t original cloud
            PrimitiveIdxPtr inliersIdx ( new PrimitiveIdx);
            if( ! cnt)  // if it is 0 use a vector with incremental integer number as starting idx (0,1,2,3,4,....)
                for( int i = 0; i< originalCloud->size(); i++)
                    inliersIdx->push_back( i);
            else 	    // else use the previous idx in the map
                inliersIdx = newInliersIdx;

            // to make the cloud smaller and look for smaller planes
            PCLCloudPtr supportCloud ( new PCLCloud);
            // remove inlier for cloud
            removePlaneInliner( iterativeCloud, inliersPlane, supportCloud);
            // recompute normals
            iterativeNorm = PCManager::estimateNormal( iterativeCloud);

            // check if the found plane is horizontal
            if( isHorizontalPlane( PCManager::estimateNormal( supportCloud), coefficientsPlane, horizontalAxis)){ // is horizontal

                // TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \\
				// check if point have relative distance above a threshold and eliminate them		\\
				// avoid that a plain contains points of separate objects just because a part of 	\\
				// them belongs to the projection of the support									\\
				// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \\

                // create new structure to be added in the output vector
                Support::Ptr supportObj( new Support);

                // set the new inlier idx w.r.t original cloud into the support output data
                newInliersIdx = createNewIdxMap( inliersIdx, inliersPlane, idxMapLayer);
                supportObj->inliers = *newInliersIdx;

                // set the cloud of the support (table)
                supportObj->support_cloud = PCManager::cloudToRosMsg( supportCloud);

                // set the cloud of the object on the table
                supportObj->on_support_cloud = PCManager::cloudToRosMsg( getPointOnPlane( supportCloud, newInliersIdx, idxMapLayer));

                // set coefficients  ( ax + by + cz + d = 0 )
                supportObj->support_coefficient_a = coefficientsPlane->values[ 0];
                supportObj->support_coefficient_b = coefficientsPlane->values[ 1];
                supportObj->support_coefficient_c = coefficientsPlane->values[ 2];
                supportObj->support_coefficient_d = coefficientsPlane->values[ 3];

                // add object to the service output vector
                res.supports_description.push_back( *supportObj);

            } else
                // manage index map in case a found plane is not horizontal
                newInliersIdx = createNewIdxMap( inliersIdx, inliersPlane, -1);

            // manage counters
            cnt++;
            idxMapLayer--;
        }
    }

    // visualize idx w.r.t. original cloud for debug
    //if( res.supportObject.size() > 0)
    // 	visualizeInlier( res.supportObject[ res.supportObject.size() - 1].inliers);


    // set used parameters
    res.used_min_iterative_cloud_percentual_size = minIterativeCloudPercentage;
    res.used_min_iterative_plane_percentual_size = minPlanePercentageSize;
    res.used_max_variance_threshold_for_horizontal = maxVarianceThForHorizontal;
    res.used_min_variance_threshold_for_horizontal =  minVarianceThForHorizontal;
    res.used_ransac_max_iteration_threshold = ransacMaxIteration;
    res.used_ransac_distance_point_in_shape_threshold = ransacThDistancePointShape;
    res.used_ransac_model_normal_distance_weigth = ransacNormalDistanceWeigth;
    res.used_horizontal_axis.push_back( horizontalAxis[ 0]);
    res.used_horizontal_axis.push_back( horizontalAxis[ 1]);
    res.used_horizontal_axis.push_back( horizontalAxis[ 2]);
    res.used_support_edge_remove_offset.push_back( supportEdgeRemoveOffset[ 0]);
    res.used_support_edge_remove_offset.push_back( supportEdgeRemoveOffset[ 1]);
    res.used_support_edge_remove_offset.push_back( supportEdgeRemoveOffset[ 2]);

    return true;
}

int main(int argc, char **argv){
    // Initialize node
    init(argc, argv, srvm::SRV_NAME_SUPPORT_FILTER);
    NodeHandle n;

    ServiceServer service = n.advertiseService( srvm::SRV_NAME_SUPPORT_FILTER, findSupports);
    spin();

    return 0;
}
