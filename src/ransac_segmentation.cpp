#include "pitt_msgs/ClustersOutput.h" // for cluster input (msg(
#include "pitt_msgs/InliersCluster.h" // for cluster input (sub msg)
#include "pitt_msgs/PrimitiveSegmentation.h" // for ransac service output
#include "pitt_msgs/ColorSrvMsg.h" // for ransac service output
#include "pitt_msgs/TrackedShapes.h" // for out message (an array of TrackedShape)
#include "pitt_msgs/TrackedShape.h" // for out message
#include "point_cloud_library/pc_manager.h" // for my static library
#include "point_cloud_library/srv_manager.h"
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>

using namespace ros;
using namespace pcm;
using namespace srvm;
//using namespace pitt_object_table_segmentation;
//using namespace geometric_tracking;
using namespace pitt_msgs;

typedef vector< InliersCluster> InliersClusters;
typedef boost::shared_ptr< InliersClusters> InliersClustersPtr;
typedef boost::shared_ptr< PrimitiveSegmentation> PrimitiveSegmentationPtr;

//definition of the type PCL with RGB information

ros::NodeHandle* nh_ptr = NULL;

boost::shared_ptr< visualization::PCLVisualizer> vis;	// to visualize cloud
boost::thread vis_thread;
boost::mutex vis_mutex;

Publisher pub; // to publish out results
//services names
const string SRV_NAME_RANSAC_SPHERE_FILTER = "sphere_segmentation_srv";
const string SRV_NAME_RANSAC_CONE_FILTER = "cone_segmentation_srv";
const string SRV_NAME_RANSAC_CYLINDER_FILTER = "cylinder_segmentation_srv";
const string SRV_NAME_RANSAC_PLANE_FILTER = "plane_segmentation_srv";
const string SRV_NAME_COLOR = "color_srv";
//ros Parameter Name
const string NAME_PARAM_CONE_MIN_INLIERS = "/pitt/srv/cone_segmentation/min_inliers";
const string NAME_PARAM_CYLINDER_MIN_INLIERS = "/pitt/srv/cylinder_segmentation/min_inliers";
const string NAME_PARAM_SPHERE_MIN_INLIERS = "/pitt/srv/sphere_segmentation/min_inliers";
const string NAME_PARAM_PLANE_MIN_INLIERS = "/pitt/srv/plane_segmentation/min_inliers";
//Default Values
static const int DEFAULT_PARAM_SPHERE_MIN_INLIERS = 40;
static const int DEFAULT_PARAM_CYLINDER_MIN_INLIERS = 40;
static const int DEFAULT_PARAM_CONE_MIN_INLIERS = 40;
static const int DEFAULT_PARAM_PLANE_MIN_INLIERS = 40;

const static float DEFAULT_CONE_OVER_CYLINDER_PRIORITY = 0.9f;// 0.90f;

static const bool DEFAULT_INPUT_PARAM_SHOW_PRIMITIVE = false;
static bool inputShowPrimitive;

const static int TXT_UNKNOWN_SHAPE_TAG = 0;
const static int TXT_PLANE_SHAPE_TAG = 1;
const static int TXT_SPHERE_SHAPE_TAG = 2;
const static int TXT_CONE_SHAPE_TAG = 3;
const static int TXT_CYLINDER_SHAPE_TAG = 4;
//name in case the service fail to be called
const string SRV_COLOR_FAILED = "no_color";
int inputSphereMinInliers, inputCylinderMinInliers, inputConeMinInliers, inputPlaneMinInliers, coneOverCylinderPriority;



void visSpin(){
    while(!vis->wasStopped()){
        boost::mutex::scoped_lock updateLock(vis_mutex);
        vis->spinOnce(100);
    }
}

// use ransac to detach sphere
bool callRansacSphereSegmentation( PCLCloudPtr cloud, PCLNormalPtr norm, PrimitiveSegmentationPtr &out){//, bool parameterTest){
    // call sphere service
    NodeHandle n;
    ServiceClient client = n.serviceClient< PrimitiveSegmentation>( SRV_NAME_RANSAC_SPHERE_FILTER);
    PrimitiveSegmentation srv;

    // set input parameters
    srv.request.cloud = PCManager::cloudToRosMsg( cloud);
    srv.request.normals = PCManager::normToRosMsg( norm);

    // call the service
    if( client.call( srv)){

        int minInliers = 0;
        nh_ptr->param(NAME_PARAM_SPHERE_MIN_INLIERS, inputSphereMinInliers, DEFAULT_PARAM_SPHERE_MIN_INLIERS);

        if(  srv.response.inliers.size() > minInliers){
            *out = srv;

            //if( parmeterTest)
            //	toWrite << "\t[sphere segmentation] " << usedParameterString( srv.response) << endl;

            return( true);
        }
    } else  ROS_ERROR( " ERROR on service %s", client.getService().c_str());
    return( false);
}
// print sphere details
void printSphereInfo( PrimitiveSegmentationPtr info, int idx){
    if( info->response.inliers.size() > 0 && info->response.coefficients.size() > 0)
        cout << idx << "- sphere found ... inliers:" << info->response.inliers.size() <<
        " Cx:" << info->response.coefficients[ 0] <<
        " Cy:" << info->response.coefficients[ 1] <<
        " Cz:" << info->response.coefficients[ 2] <<
        " radius:" << info->response.coefficients[ 3] << endl;
}

// use ransac to detach cylinder
bool callRansacCylinderSegmentation( PCLCloudPtr cloud, PCLNormalPtr norm, PrimitiveSegmentationPtr &out){//, bool parmeterTest){
    // call sphere service
    NodeHandle n;
    ServiceClient client = n.serviceClient< PrimitiveSegmentation>( SRV_NAME_RANSAC_CYLINDER_FILTER);
    PrimitiveSegmentation srv;

    // set input parameters
    srv.request.cloud = PCManager::cloudToRosMsg( cloud);
    srv.request.normals = PCManager::normToRosMsg( norm);

    // call the service
    if( client.call( srv)){
        int minInliers = 0;
        nh_ptr->param(NAME_PARAM_CYLINDER_MIN_INLIERS, inputCylinderMinInliers, DEFAULT_PARAM_CYLINDER_MIN_INLIERS);
        if(  srv.response.inliers.size() > minInliers){
            *out = srv; // get respose

            //if( parmeterTest)
            //	toWrite << "\t[cylinder segmentation] " << usedParameterString( srv.response) << endl;

            return( true);
        }
    } else  ROS_ERROR( " ERROR on service %s", client.getService().c_str());
    return( false);
}
// print sphere details
void printCylinderInfo( PrimitiveSegmentationPtr info, int idx){
    if( info->response.inliers.size() > 0 && info->response.coefficients.size() > 0)
        cout << " cylinder found ... inliers:" << info->response.inliers.size() <<
        " pointX:" << info->response.coefficients[ 0] <<
        " pointY:" << info->response.coefficients[ 1] <<
        " pointZ:" << info->response.coefficients[ 2] <<
        "  axisX:" << info->response.coefficients[ 3] <<
        "  axisY:" << info->response.coefficients[ 4] <<
        "  axisZ:" << info->response.coefficients[ 5] <<
        " radius:" << info->response.coefficients[ 6] << endl;
}

// use ransac to detach cone
bool callRansacConeSegmentation( PCLCloudPtr cloud, PCLNormalPtr norm, PrimitiveSegmentationPtr &out){//,  bool parmeterTest){
    // call sphere service
    NodeHandle n;
    ServiceClient client = n.serviceClient< PrimitiveSegmentation>( SRV_NAME_RANSAC_CONE_FILTER);
    PrimitiveSegmentation srv;

    // set input parameters
    srv.request.cloud = PCManager::cloudToRosMsg( cloud);
    srv.request.normals = PCManager::normToRosMsg( norm);

    // call the service
    if( client.call( srv)){
        int minInliers = 0;
        nh_ptr->param(NAME_PARAM_CONE_MIN_INLIERS, inputConeMinInliers, DEFAULT_PARAM_CONE_MIN_INLIERS);
        if(  srv.response.inliers.size() > minInliers){
            *out = srv; // get the respose

            //if( parmeterTest)
            //	toWrite << "\t[cone segmentation] " << usedParameterString( srv.response) << endl;
            return( true);
        }
    } else  ROS_ERROR( " ERROR on service %s", client.getService().c_str());
    return( false);
}

// print sphere details
void printConeInfo( PrimitiveSegmentationPtr info, int idx){
    if( info->response.inliers.size() > 0 && info->response.coefficients.size() > 0)
        cout << " cone  found ... inliers:" <<  info->response.inliers.size() <<
        " apexX:" << info->response.coefficients[ 0] <<
        " apexY:" << info->response.coefficients[ 1] <<
        " apexZ:" << info->response.coefficients[ 2] <<
        " axixX:" << info->response.coefficients[ 3] <<
        " axixY:" << info->response.coefficients[ 4] <<
        " axixZ:" << info->response.coefficients[ 5] <<
        " angle:" << info->response.coefficients[ 6] << endl;
}


// use ransac to detach plane
bool callRansacPlaneSegmentation( PCLCloudPtr cloud, PCLNormalPtr norm, PrimitiveSegmentationPtr &out){//,  bool parmeterTest){
    // call sphere service
    NodeHandle n;
    ServiceClient client = n.serviceClient< PrimitiveSegmentation>( SRV_NAME_RANSAC_PLANE_FILTER);
    PrimitiveSegmentation srv;

    // set input parameters
    srv.request.cloud = PCManager::cloudToRosMsg( cloud);
    srv.request.normals = PCManager::normToRosMsg( norm);

    // call the service
    if( client.call( srv)){
        int minInliers = 0;
        nh_ptr->param(NAME_PARAM_PLANE_MIN_INLIERS, inputPlaneMinInliers, DEFAULT_PARAM_PLANE_MIN_INLIERS);
        if(  srv.response.inliers.size() > minInliers){
            *out = srv; // get response

            //if( parmeterTest)
            //	toWrite << "\t[plane segmentation] " << usedParameterString( srv.response) << endl;

            return( true);
        }
    } else ROS_ERROR( " ERROR on service %s", client.getService().c_str());
    return( false);
}
//function which calls the color service
bool callColorSrv(PCLCloudPtr cloud, string* color){
    NodeHandle n;
    ServiceClient client = n.serviceClient<ColorSrvMsg>(SRV_NAME_COLOR);
    ColorSrvMsg srv;
    float R,G,B;
    srv.request.cloud=PCManager::cloudToRosMsg( cloud);
    if(client.call(srv))
    {
        *color=srv.response.Color.data.c_str();
        ROS_INFO_STREAM(srv.response.Color.data.c_str()<<"color_name"<<endl);
        R=srv.response.redPercentage.data;

        return (true);
    }
    else
    {
        return(false);
    }
}
// print sphere details
void printPlaneInfo( PrimitiveSegmentationPtr info, int idx){
    if( info->response.inliers.size() > 0 && info->response.coefficients.size() > 0)
        cout << " cone  found ... inliers:" <<  info->response.inliers.size() <<
        " aX:" << info->response.coefficients[ 0] <<
        " bY:" << info->response.coefficients[ 1] <<
        " cZ:" << info->response.coefficients[ 2] <<
        " d :" << info->response.coefficients[ 3] << endl;
}

string returnPrimitiveNameFromTag( int primitiveTag){
    switch( primitiveTag){
        case TXT_UNKNOWN_SHAPE_TAG:  return( "unknown");
        case TXT_PLANE_SHAPE_TAG:	 return( "Plane");
        case TXT_SPHERE_SHAPE_TAG:	 return( "Sphere");
        case TXT_CONE_SHAPE_TAG:	 return( "Cone");
        case TXT_CYLINDER_SHAPE_TAG: return( "Cylinder");
    }
}


// node callback
string centroidFileLog;
void clustersAcquisition(const ClustersOutputConstPtr& clusterObj){
    // iterate over all the input clusters
    InliersClusters clusters = clusterObj->cluster_objs;
    // create an output vector to be published
    TrackedShapes::Ptr outShapes ( new TrackedShapes);
    if( clusters.size() > 0){ // at least one cluster
        // add the response to the shape map of the point cloud manager
        for( int j = 0; j < clusters.size(); j++){ // scan all the clusters
            // add primitive
            PCLCloudPtr cluster = PCManager::cloudForRosMsg( clusters[ j].cloud);

            PCLNormalPtr normalCluster = PCManager::estimateNormal( cluster);

            // initialize inlier count
            size_t planeInl = 0, coneInl = 0, sphereInl = 0, cylinderInl = 0;
            size_t clusterSize = cluster->size();

            // call ransac sphere service
            PrimitiveSegmentationPtr outSphere ( new PrimitiveSegmentation);
            if( callRansacSphereSegmentation( cluster, normalCluster, outSphere))
                sphereInl = outSphere->response.inliers.size();

            // call ransac cylinder service
            PrimitiveSegmentationPtr outCylinder ( new PrimitiveSegmentation);
            if( callRansacCylinderSegmentation( cluster, normalCluster, outCylinder))
                cylinderInl = outCylinder->response.inliers.size();


            // call ransac cone service
            PrimitiveSegmentationPtr outCone ( new PrimitiveSegmentation);
            if( callRansacConeSegmentation( cluster, normalCluster, outCone))
                coneInl = outCone->response.inliers.size();

            // call ransac plane service
            PrimitiveSegmentationPtr outPlane ( new PrimitiveSegmentation);
            if( callRansacPlaneSegmentation( cluster, normalCluster, outPlane))
                planeInl = outPlane->response.inliers.size();

            // show cluster with color w.r.t. primitive
            int R, G, B;
            int detechedPrimitiveTag;
            float xC, yC, zC;
            string color;
            //call color service

            PrimitiveSegmentationPtr primitiveInfo;
            if( ( ! planeInl) && ( ! sphereInl) && ( ! cylinderInl) && ( ! coneInl)){
                R = 100, G = 100, B = 100;	// nothing : GRAY
                detechedPrimitiveTag = TXT_UNKNOWN_SHAPE_TAG;
            } else if ( ( coneInl >= planeInl) && ( coneInl >= sphereInl) &&
                    ( coneInl >= cylinderInl * DEFAULT_CONE_OVER_CYLINDER_PRIORITY)) {
                R = 0; G = 255; B = 0;		// cone : GREEN	[priority w.r.t. cylinder]
                detechedPrimitiveTag = TXT_CONE_SHAPE_TAG;
                primitiveInfo = outCone;
                xC = outCone->response.x_centroid;
                yC = outCone->response.y_centroid;
                zC = outCone->response.z_centroid;
            } else if ( ( cylinderInl >= planeInl) && ( cylinderInl >= coneInl) && ( cylinderInl >= sphereInl)) {
                R = 255; G = 255; B = 255;  // cylinder : WHITE
                detechedPrimitiveTag = TXT_CYLINDER_SHAPE_TAG;
                primitiveInfo = outCylinder;
                xC = outCylinder->response.x_centroid;
                yC = outCylinder->response.y_centroid;
                zC = outCylinder->response.z_centroid;
                //centroidFileLog += boost::to_string( j) + "," + boost::to_string( outCylinder->response.xCentroid) + "," + boost::to_string( outCylinder->response.yCentroid) + "," + boost::to_string( outCylinder->response.zCentroid) + "\n";
            }else if( ( planeInl >= coneInl) && ( planeInl >= sphereInl) && ( planeInl >= cylinderInl)){
                R = 255; G = 0; B = 0;		// plane : RED
                detechedPrimitiveTag = TXT_PLANE_SHAPE_TAG;
                primitiveInfo = outPlane;
                xC = outPlane->response.x_centroid;
                yC = outPlane->response.y_centroid;
                zC = outPlane->response.z_centroid;
            } else if ( ( sphereInl >= planeInl) && ( sphereInl >= coneInl) && ( sphereInl >= cylinderInl)) {
                R = 0; G = 0; B = 255;		// sphere : BLUE [priority w.r.t. cone]
                detechedPrimitiveTag = TXT_SPHERE_SHAPE_TAG;
                primitiveInfo = outSphere;
                xC = outSphere->response.x_centroid;
                yC = outSphere->response.y_centroid;
                zC = outSphere->response.z_centroid;
                //centroidFileLog += boost::to_string( j) + "," + boost::to_string( outSphere->response.xCentroid) + "," + boost::to_string( outSphere->response.yCentroid) + "," + boost::to_string( outSphere->response.zCentroid) + "\n";
            } else {
                R = 100, G = 100, B = 100;	// nothing : GRAY
                detechedPrimitiveTag = TXT_UNKNOWN_SHAPE_TAG;
            }

            if( inputShowPrimitive){
                boost::mutex::scoped_lock lock(vis_mutex);
                PCManager::updateVisor( vis, cluster, R, G, B, "clusterShape" + boost::to_string( j));

                string log_str = str(boost::format("INLIERS: %s/%s/%s/%s    CONE/CYLINDER PRIORITY:%s")
                                     %inputSphereMinInliers %inputCylinderMinInliers %inputConeMinInliers
                                     %inputPlaneMinInliers %DEFAULT_CONE_OVER_CYLINDER_PRIORITY);
                vis->updateText(log_str, 10, 520, "log_str_depth");
            }

            ROS_INFO( "cluster_%d: %d #INLIER plane: %d sphere: %d cylinder: %d cone: %d selected: %s",
                      clusters[ j].shape_id, (int) cluster->size(), planeInl, sphereInl, cylinderInl, coneInl, returnPrimitiveNameFromTag( detechedPrimitiveTag).c_str());

            // prepared output to be published
            TrackedShape::Ptr outShape ( new TrackedShape);
            outShape->object_id = clusters[ j].shape_id;
            outShape->x_pc_centroid = clusters[ j].x_centroid;
            outShape->y_pc_centroid = clusters[ j].y_centroid;
            outShape->z_pc_centroid = clusters[ j].z_centroid;
            outShape->shape_tag = returnPrimitiveNameFromTag( detechedPrimitiveTag);
            // calling the color service
            if(callColorSrv(cluster, &color)) {
                outShape->color.data= color;
            } else
            {
                outShape->color.data=SRV_COLOR_FAILED;

            }
            if( detechedPrimitiveTag != TXT_UNKNOWN_SHAPE_TAG){
                outShape->x_est_centroid = xC;
                outShape->y_est_centroid = yC;
                outShape->z_est_centroid = zC;
                outShape->coefficients = primitiveInfo->response.coefficients;
            }
            outShapes->tracked_shapes.push_back( *outShape);
            // propagate usefull data from geometric_tracker (no computation on those vectors)
            // not working !!!!!
            //outShapes->clusterAdded = clusterObj->clusterAdded;
            //outShapes->clusterAdded = clusterObj->clusterRemove;
        }
    } // error in cluster service
    ROS_INFO( " ------------------------------------ " );
    //PCManager::writeToFile( centroidFileLog, "/home/luca-phd/ransac_centroid");

    // publish out results
    pub.publish( outShapes);

}


// main method of the node
int main(int argc, char **argv){

    init(argc, argv, "ransac_segmentation");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    centroidFileLog = "";
    inputShowPrimitive = srvm::getBoolPtrParameter( argv[ 1], DEFAULT_INPUT_PARAM_SHOW_PRIMITIVE);

    // subscribe to cluster published by obj_segmentation.cpp
    //Subscriber sub = n.subscribe ("obj_segmentation/cluster", 10, clustersAcquisition); // get data from cluster node
    Subscriber sub = nh.subscribe( "geometric_tracker/trackedCluster", 10, clustersAcquisition); // get data from tracker node
    // to publish the data at the end of the process
    pub = nh.advertise< TrackedShapes>( "ransac_segmentation/trackedShapes", 10);

    string log_str = str(boost::format("INLIERS: %s/%s/%s/%s    CONE/CYLINDER PRIORITY:%s")
                         %DEFAULT_PARAM_SPHERE_MIN_INLIERS %DEFAULT_PARAM_CYLINDER_MIN_INLIERS %DEFAULT_PARAM_CONE_MIN_INLIERS
                         %DEFAULT_PARAM_PLANE_MIN_INLIERS %DEFAULT_CONE_OVER_CYLINDER_PRIORITY);

    // create window to visualize cloud
    if( inputShowPrimitive) {
        vis = PCManager::createVisor("Ransac shape segmentation");
        vis->setCameraPosition(8-2.19051, 0.198678, 0.366248, -0.044886, 0.0859204, 0.471681, -0.0487582, 0.00610776, 0.998792);
        vis->setCameraFieldOfView(0.8575);
        vis->setCameraClipDistances(0.0064556, 6.4556);
        vis->setPosition(1, 480);
        vis->setSize(960, 540);
        vis->addText(log_str, 10, 520, 13, 0.9, 0.9, 0.9, "log_str_depth");
        vis_thread = boost::thread(visSpin);
    }


    //Rate rate(20); // hz
    //ros::Rate r(20);
    while ( nh.ok()){
        spinOnce();
        //r.sleep();
    }
    if (inputShowPrimitive){
        vis->close();
        vis_thread.join();
    }
    return 0;
}
