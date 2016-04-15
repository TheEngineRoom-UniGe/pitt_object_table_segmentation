#include <ros/ros.h>
// transform frame library include
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/crop_box.h>
// for system PCL static library
#include "../point_cloud_library/pc_manager.h"
#include "../point_cloud_library/srv_manager.h"
// custom message to get data and send results
#include <pitt_msgs/ArmFilter.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// use also abbreviations (typedef: PCLCloud, PCLCloudPtr) (given from PCManager in turn given from PCPrimitive)
using namespace pcl;
using namespace std;
using namespace tf;
using namespace pcm;
using namespace srvm;
using namespace ros;
using namespace Eigen;
using namespace sensor_msgs;
using namespace pitt_msgs;

const Duration WAIT_FOR_TF_TIME_OUT = Duration( srvm::DEFAULT_TF_WAIT_SECONDS);

//transformation vector and matrices
StampedTransform leftForearmCameraTansf, rightForearmCameraTansf, leftElbowCameraTansf, rightElbowCameraTansf;

//true if an error in retrieve a transformation occurs
bool tfError = false;
bool showClouds;
// to visualize cloud
boost::shared_ptr< visualization::PCLVisualizer> vis;
boost::thread vis_thread;
boost::mutex vis_mutex;

void visSpin(){
    vis->spin();
}

//procedure to remove the points belonging to the arm in the input cloud
PCLCloudPtr armFiltering(PCLCloudPtr original, Vector4f minValues, Vector4f maxValues, StampedTransform frame){
	// Initialize useful quantities
	double roll, pitch, yaw;
	Vector3f translation;
	Vector3f rotation;
	Affine3f trans = Affine3f::Identity();
	CropBox<PointXYZ> cropFilter;
	PCLCloudPtr filteredCloud (new PCLCloud);
	tf::Matrix3x3 rotMat;

	// get the transformation
	tf::Quaternion rotQuat = frame.getRotation();
	rotMat.setRotation( rotQuat);
	rotMat.getRPY( roll, pitch, yaw);

	// get the translation of the transformation
	translation[0] = frame.getOrigin().getX();
	translation[1] = frame.getOrigin().getY();
	translation[2] = frame.getOrigin().getZ();

	// get the orientation of the transformation
	rotation[0] = roll;
	rotation[1] = pitch;
	rotation[2] = yaw;

	// remove from the cloud the points that are inside the bounding box
	cropFilter.setInputCloud( original);
	cropFilter.setMin( minValues);
	cropFilter.setMax( maxValues);
	cropFilter.setTranslation( translation);
	cropFilter.setRotation( rotation);
	cropFilter.setTransform( trans);
	cropFilter.setNegative( true);
	cropFilter.filter( *filteredCloud);

	// return the cloud without the points that are belonging to the robor arm
	return filteredCloud;
}

Vector4f generateBoxVector( vector< float> vec){
	Vector4f out;
	out << vec[ 0], vec[ 1], vec[ 2], 1;
	return out;
}

//service function
bool filter( ArmFilterRequest& input, ArmFilterResponse& output){
	//convert ROS point cloud to PCL format
	PCLCloudPtr inputCloud( new PCLCloud);
	fromROSMsg( input.input_cloud, *inputCloud);

	// get bounding box parameters
	vector<float> minForearm = srvm::getService3DArrayParameter( input.forearm_bounding_box_min_value, srvm::DEFAULT_PARAM_ARM_SRV_MIN_FOREARM_BOX);
	vector<float> maxForearm = srvm::getService3DArrayParameter( input.forearm_bounding_box_max_value, srvm::DEFAULT_PARAM_ARM_SRV_MAX_FOREARM_BOX);
	vector<float> minElbow = srvm::getService3DArrayParameter( input.elbow_bounding_box_min_value, srvm::DEFAULT_PARAM_ARM_SRV_MIN_ELBOW_BOX);
	vector<float> maxElbow = srvm::getService3DArrayParameter( input.elbow_bounding_box_max_value, srvm::DEFAULT_PARAM_ARM_SRV_MAX_ELBOW_BOX);

	// build the bounding boxes
	// for forearm link
	Vector4f forearmMinValue = generateBoxVector( minForearm);
	Vector4f forearmMaxValue = generateBoxVector( maxForearm);
	// for elbow link
	Vector4f elbowMinValue = generateBoxVector( minElbow);
	Vector4f elbowMaxValue = generateBoxVector( maxElbow);

	//initialize output
	PCLCloudPtr outputCloud1( new PCLCloud);
	PCLCloudPtr outputCloud2( new PCLCloud);
	PCLCloudPtr outputCloud3( new PCLCloud);
	PCLCloudPtr outputCloud4( new PCLCloud);

	//check transform availability
	if( ! tfError){
		int leftForearmRemovedCnt, rightForearmRemovedCnt, leftElbowRemovedCnt, rightElbowRemovedCnt; // used only for logs
		// remove the points that are into the bounding box around the arms.
		outputCloud1 = armFiltering( inputCloud, forearmMinValue, forearmMaxValue, leftForearmCameraTansf);
		leftForearmRemovedCnt = inputCloud->size() - outputCloud1->size();
		outputCloud2 = armFiltering( outputCloud1, forearmMinValue, forearmMaxValue, rightForearmCameraTansf);
		rightForearmRemovedCnt = outputCloud1->size() - outputCloud2->size();
		outputCloud3 = armFiltering( outputCloud2, elbowMinValue, elbowMaxValue, leftElbowCameraTansf);
		leftElbowRemovedCnt = outputCloud2->size() - outputCloud3->size();
		outputCloud4 = armFiltering( outputCloud3, elbowMinValue, elbowMaxValue, rightElbowCameraTansf);
		rightElbowRemovedCnt = outputCloud3->size() - outputCloud4->size();
		// produce filtering logs
		ROS_INFO_STREAM( "Arm filtering performed with parameters and results:" << endl
				<< "\tForearm (min:" << getArrayToPrint( minForearm) << ") (max:" << getArrayToPrint( maxForearm) << "). "
				<< "Removed points (left:" << leftForearmRemovedCnt << ") (right:" << rightForearmRemovedCnt << ")" << endl
				<< "\tElbow (min:" << getArrayToPrint( minElbow) << ") (max:" << getArrayToPrint( maxElbow) << "). "
				<< "Removed points (left:" << leftElbowRemovedCnt << ") (right:" << rightElbowRemovedCnt << ")");
	} else {
		// do not filter since error on frame transformation (returns the input cloud)
		ROS_ERROR( "Error on retrieving the transformation between frames on arm filtering. No operation performed (input_Cloud = outputCloud).");
		outputCloud4 = inputCloud;
	}

	// eventually show clouds for debugging and tuning
	if( showClouds){
        boost::mutex::scoped_lock lock(vis_mutex);
		PCManager::updateVisor( vis, inputCloud, 255, 0, 0, "original");
		PCManager::updateVisor( vis, outputCloud4, 0, 255, 0, "filtered");
	}

	//preparing ROS output message
	PointCloud2Ptr temp(new PointCloud2);
	toROSMsg( *outputCloud4, *temp);
	output.armless_cloud = *temp;
	output.used_forearm_bounding_box_min_value = minForearm;
	output.used_forearm_bounding_box_max_value = maxForearm;
	output.used_elbow_bounding_box_min_value = minElbow;
	output.used_elbow_bounding_box_max_value = maxElbow;

	return true;
}


//main node procedure
int main(int argc, char **argv){
	//Initialize node
	init( argc, argv, srvm::SRV_NAME_ARM_FILTER);
	NodeHandle n;
	ServiceServer service = n.advertiseService( srvm::SRV_NAME_ARM_FILTER, filter);

	//get input parameter
	int numberOfInputParameter = 6;

	string cameraFrameName, rightForearmFrameName, leftForearmFrameName, rightElbowFrameName, leftElbowFrameName;
	if( argc == numberOfInputParameter + 1){ // args[ 0] is the path to the executable file
		showClouds = srvm::getBoolPtrParameter( argv[1], srvm::DEFAULT_PARAM_ARM_SRV_SHOW_CLOUDS);
		// read the name of the camera frame
		cameraFrameName = srvm::getStringPtrParameter( argv[ 2], srvm::DEFAULT_PARAM_ARM_SRV_CAMERA_FRAME);
		// read the name of the robotic arm frmaes
		leftForearmFrameName = srvm::getStringPtrParameter( argv[ 3], srvm::DEFAULT_PARAM_ARM_SRV_LEFT_FOREARM_FRAME);
		rightForearmFrameName = srvm::getStringPtrParameter( argv[ 4], srvm::DEFAULT_PARAM_ARM_SRV_RIGHT_FOREARM_FRAME);
		leftElbowFrameName = srvm::getStringPtrParameter( argv[ 5], srvm::DEFAULT_PARAM_ARM_SRV_LEFT_ELBOW_FRAME);
		rightElbowFrameName = srvm::getStringPtrParameter( argv[ 6], srvm::DEFAULT_PARAM_ARM_SRV_RIGHT_ELBOW_FRAME);
	} else { // set all to defaults
		cameraFrameName = srvm::DEFAULT_PARAM_ARM_SRV_CAMERA_FRAME;
		rightForearmFrameName = srvm::DEFAULT_PARAM_ARM_SRV_RIGHT_FOREARM_FRAME;
		leftForearmFrameName = srvm::DEFAULT_PARAM_ARM_SRV_LEFT_FOREARM_FRAME;
		rightElbowFrameName = srvm::DEFAULT_PARAM_ARM_SRV_RIGHT_ELBOW_FRAME;
		leftElbowFrameName = srvm::DEFAULT_PARAM_ARM_SRV_LEFT_ELBOW_FRAME;
		showClouds = srvm::DEFAULT_PARAM_ARM_SRV_SHOW_CLOUDS;
		ROS_WARN_STREAM( "input parameter given to \"" << srvm::SRV_NAME_ARM_FILTER << "\" are not correct. Setting all to the default value.");
	}
	// log the value coming from using inputs
	ROS_INFO_STREAM( srvm::SRV_NAME_ARM_FILTER << " initialised with:" 		<< endl
					<< "\t camera frame: \t\""				<< cameraFrameName 			<< "\""	<< endl
					<< "\t right forearm frame: \t\"" 		<< rightForearmFrameName 	<< "\""	<< endl
					<< "\t left forearm frame: \t\""		<< leftForearmFrameName 	<< "\""	<< endl
					<< "\t right elbow frame: \t\""			<< rightElbowFrameName		<< "\""	<< endl
					<< "\t left elbow frame: \t\""			<< leftElbowFrameName 		<< "\"");

	// eventually show filtered and not clouds
	if( showClouds) {
        vis = PCManager::createVisor("Arm Filtering");
        vis->setCameraPosition(8.6096e-05, 0.61526, 0.0408496, 0, 0, 1, 0.0230758, -0.841489, -0.539782);
        vis->setCameraFieldOfView(0.8575);
        vis->setCameraClipDistances(0.00433291,4.33291);
        vis->setPosition(1,1);
        vis->setSize(960,540);
        vis_thread = boost::thread(visSpin);
    }

	// set the listener to the frame transformations
	TransformListener* baxter_tf_listener = new( TransformListener);
	//ros::Rate r(20);
	while ( n.ok()){
		try{
			// camera to left forearm transformation
			baxter_tf_listener->waitForTransform(cameraFrameName, leftForearmFrameName, Time(0), WAIT_FOR_TF_TIME_OUT);
			baxter_tf_listener->lookupTransform(cameraFrameName, leftForearmFrameName, Time(0), leftForearmCameraTansf);
			// camera to right forearm transformation
			baxter_tf_listener->waitForTransform(cameraFrameName, rightForearmFrameName, Time(0), WAIT_FOR_TF_TIME_OUT);
			baxter_tf_listener->lookupTransform(cameraFrameName, rightForearmFrameName, Time(0), rightForearmCameraTansf);
			// camera to left elbow transformation
			baxter_tf_listener->waitForTransform(cameraFrameName, leftElbowFrameName, Time(0), WAIT_FOR_TF_TIME_OUT);
			baxter_tf_listener->lookupTransform(cameraFrameName, leftElbowFrameName, Time(0), leftElbowCameraTansf);
			// camera to right elbow transformation
			baxter_tf_listener->waitForTransform(cameraFrameName, rightElbowFrameName, Time(0), WAIT_FOR_TF_TIME_OUT);
			baxter_tf_listener->lookupTransform(cameraFrameName, rightElbowFrameName, Time(0), rightElbowCameraTansf);
			// reset error flag for filtering service
			tfError = false;
		} catch (TransformException &ex){
			ROS_WARN( "%s",ex.what());
			tfError = true; // set error flag for filtering service
			continue;
		}
		spinOnce();
		//r.sleep();
	}
    vis_thread.join();
	return 0;
}

