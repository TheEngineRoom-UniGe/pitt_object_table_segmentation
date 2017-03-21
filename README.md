###Obj_segmentation.cpp

**Main parameters ** 
All the default values  are defined in the "srv_manager.h" file .
Convention used : 
Default Value : DEFAULT_INPUT_PARAM_Name.
Variable name:  inputName
Example 
Name: RawCloudTopic
Default Value: DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC
Variable Name: inputRawCloudTopic
Index |Name |Default | Description|
---  | --- | --- | --- |
1|RawCloudTopic |"/camera/depth/points"|Subscribe to this topic to collect kinect depth points|
2|ShowOriginalCloud|false|Decide whether visualize the original point Clouds|
3|ShowSupportClouds|false|Decide whether visualize the support clouds|
4|ShowClusterClouds|false|Decide whether visualize the cluster clouds|
5|ShowObjectOnSupport|false|Decide whether visualize the object on support clouds|
6|CentroidLogFilePath|" "|Path in which save the file |

**ROS parameters **
All the default values and the ros name are defined in the "srv_manager.h" file .
Convention used :
Ros Name:  NAME_PARAM_Name
Default Value : DEFAULT_PARAM_Name
Variable name:  inputName
Example 
Name: CloudReferenceFrame
Ros Name : NAME_PARAM_INPUT_CLOUD_REFERENCE_FRAME
Default Value: DEFAULT_PARAM_INPUT_CLOUD_REFERENCE_FRAME 
Variable Name: inputCloudReferenceFrame

In order to automatically set the default values the user can set all the parameters to "-1" if they are either float or int. In case of string parameter they have to be set to " . " .

 Name  |Launch Name |Default | Description|
---  | --- |--- | --- |
CloudReferenceFrame |"/pitt/ref_frame/input_cloud" |"/camera_depth_optical_frame"|Reference frame for the input clouds, used to obtain the transformation between the camera and the desired frame  |
outputCloudReferenceFrame|"/pitt/ref_frame/output_cloud"|"/world"|Reference frame for the input clouds, used to obtain the transformation between the camera and the desired frame |
srvDeepFilterDepthThreshold|"/pitt/service/deep_filter/z_threshold"|3.000f|Parameter for the request of the deepfilter service, the service will get rid of the point farther than this threshold along the z axis|
 srvArmFilterForearmMinBox|"/pitt/srv/arm_filter/min_forearm_box"|-0.040f, -0.120f, -0.190f|The minimum coordinates for the forearm bounding box w.r.t. the input (left/right) forearm frame |
 srvArmFilterForearmMaxBox|"/pitt/srv/arm_filter/max_forearm_box"|0.340,  0.120,  0.105|The maximum coordinates for the forearm bounding box w.r.t. the input (left/right) forearm frame|
 srvArmFilterElbowMinBox|"/pitt/srv/arm_filter/min_elbow_box"|-0.090, -0.135, -0.160|The minimum coordinates for the elbow bounding box w.r.t. the input (left/right) elbow frame|
srvArmFilterElbowMaxBox|"/pitt/srv/arm_filter/max_elbow_box"|0.440,  0.135,  0.110|The maximum coordinates for the elbow bounding box w.r.t. the input (left/right) elbow frame|
srvSupportMinIterativeCloudPercentage|"/pitt/srv/supports_segmentation/min_iter_cloud_percent"|0.030f|Minimum percentage to stop looking for supports|
srvSupportMinIterativeSupportPercentage|"/pitt/srv/supports_segmentation/min_iter_support_percent"|0.030f|Minimum number of found planes to stop looking for support|
srvSupportHorizzontalVarianceTreshold|"/pitt/srv/supports_segmentation/horizontal_variance_th"|0.09f|the value(cross product) used to discriminate whether a support is parallel to the ground|
 srvSupportRansacInShapeDistancePointThreshold|"/pitt/srv/supports_segmentation/in_shape_distance_th"|0.02f|RANSAC parameter, distance between points (in meters) to  consider them belonging to the same shape|
srvSupportRansacMaxIterationThreshold|"/pitt/srv/supports_segmentation/max_iter"|10|RANSAC parameter maximum number of iterations for plane segmentation on support service|
srvSupportHorizontalAxis|"/pitt/srv/supports_segmentation/horizontal_axis"|{ 0.0f, 0.0f, -1.0f}|Axis used to check whether the plane is horizontal|
srvSupportEdgeRemoveOffset|"/pitt/srv/supports_segmentation/edge_remove_offset"|{ 0.02, 0.02, 0.005}|Parameter used to remove some space at the edges of the table in order not to consider objects attached to it (for instance walls)|


**Subscription **
Topic  |Callback |Msg | Description|
---  | --- |--- | --- |
RawCloudTopic|depthAcquisition|PointCloud2Ptr|Kinect PointCloud to be processed|

**Publication**
Topic  |Msg | Description|
---  | --- |--- | 
TOPIC_OUT_NAME_OBJECT_PERCEPTION (defined in the "srv_manager.h")|ClustersOutput|The clusters of the point clouds are published in this node. It is used to comunicate with the other nodes (geometric tracker)

###ransac_segmentation.cpp

**Main parameters ** 
All the default values are defined in the cpp itself 
Convention used : 
Default Value : DEFAULT_INPUT_PARAM_Name.
Variable name:  inputName
Index |Name |Default | Description|
---  | --- | --- | --- |
1|ShowPrimitive|false|Decide whether show the point clouds|
**ROS parameters **
All the default values and the ros name  are defined in the cpp file itself.
Convention used :
Ros Name:  NAME_PARAM_Name
Default Value : DEFAULT_PARAM_Name
Variable name:  inputName


 Name  |Launch Name |Default | Description|
---  | --- |--- | --- |
SphereMinInliers|"/pitt/srv/sphere_segmentation/min_inliers"|40|Minimum inliers that must be found in order to consider the possibility of having a sphere|
CylinderMinInliers|"/pitt/srv/cylinder_segmentation/min_inliers"|40|Minimum inliers that must be found in order to consider the possibility of having a cylinder |
ConeMinInliers|"/pitt/srv/cone_segmentation/min_inliers"|40|Minimum inliers that must be found in order to consider the possibility of having a cone|
PlaneMinInliers|"/pitt/srv/plane_segmentation/min_inliers"|40|Minimum inliers that must be found in order to consider the possibility of having a palne|

**Subscription **
Topic  |Callback |Msg | Description|
---  | --- |--- | --- |
"geometric_tracker/trackedCluster"|clustersAcquisition|ClustersOutputConstPtr|Tracked clusters to be processed. 

**Publication**
Topic  |Msg | Description|
---  | --- |--- | 
"ransac_segmentation/trackedShapes"|TrackedShapes|Published the tracked shapes|




###Arm_Filter_srv.cpp

**Main parameters ** 
All the default values are defined in the "srv_manager.h" file .
Convention used : 
Default Value : DEFAULT_PARAM_ARM_SRV_Name.
Variable name:  inputArmSrvName
ex
Name: ShowClouds
Default Value DEFAULT_PARAM_ARM_SRV_SHOW_CLOUDS
Variable Name inputArmSrvShowClouds
     
Index |Name |Default | Description|
:---  | :--- | :--- | :--- |
1| ShowClouds|false|Decide whether visualize the  point Clouds|
2|CameraFrameName|"/camera_depth_optical_frame"|Name of the camera frame|
3|LeftForearmFrameName|"/left_lower_forearm"|Name of the left Forearm frame|
4|RightForearmFrameName|"/right_lower_forearm"|Name of the right Elbow frame|
5|LeftElbowFrameName | "/left_lower_elbow"|Name of the left elbow frame|
6|RightElbowFrameName|"/right_lower_elbow"|name of the right elbow frame |

**Msg **
**input**
Type |Name| Description|
:---  | :--- |:--- | 
sensor_msgs/PointCloud2 | input_cloud| The original input cloud|
float32[]| forearm_bounding_box_min_value|[X,Y,Z] the coordinate of the minimum point of the box around the forearm (w.r.t. the center of mass of the box) |
float32[]|forearm_bounding_box_max_value| [X,Y,Z] the coordinate of the maximum point of the box around the forearm (w.r.t. the center of mass of the box)|
float32[]| elbow_bounding_box_min_value	|[X,Y,Z] the coordinate of the minimum point of the box around the elbow (w.r.t. the center of mass of the box)|
float32|elbow_bounding_box_max_value|[X,Y,Z] the coordinate of the maximum point of the box around the elbow (w.r.t. the center of mass of the box)|
**output**
Type |Name| Description|
:---  | :--- |:--- | 
sensor_msgs/PointCloud2 |armless_cloud	|The output cloud. Where the points of the robot arm are removed
float32[]|used_forearm_bounding_box_min_value|Returns back the value of the parameter used |
float32[]|used_forearm_bounding_box_max_value|Returns back the value of the parameter used|
float32[] |used_elbow_bounding_box_min_value|Returns back the value of the parameter used |
float32[] |used_elbow_bounding_box_max_value|Returns back the value of the parameter used |


###Cluster_srv.cpp

**Ros parameter ** 
All the default values and the ros name are defined in the cpp file itself .
Convention used : 
Default Value : DEFAULT_PARAM_CLUSTER_SRV_Name
Ros name :NAME_PARAM_CLUSTER_SRV_Name.
Variable name:  inputClusterSrvName
Example 
Name : Tollerance
Default Value : DEFAULT_PARAM_CLUSTER_SRV_TOLLERANCE
Ros Name :NAME_PARAM_CLUSTER_SRV_TOLLERANCE
Variable name : inputClusterSrvTollerance

 Name  |Launch Name |Default | Description|
---  | --- |--- | --- |
Tolerance|"/pitt/srv/cluster_segmentation/tolerance" |0.03 |Distance (in meters) needed to consider two points belonging to the same cluster|
MinSizeRate|"/pitt/srv/cluster_segmentation/min_rate"|0.01|Min percentage wrt the total number of points to consider points as a cluster |
MaxSizeRate |"/pitt/srv/cluster_segmentation/max_rate"|0.99 |Max percentage wrt the total number of points to consider points as a cluster|
MinInputSize| "/pitt/srv/cluster_segmentation/min_input_size"| 30|Minimum number of points that  must be present in the input cloud in order to process it|


**Msg **
**input**
Type |Name| Description|
:---  | :--- |:--- | 
sensor_msgs/PointCloud2| cloud |Input cloud 
**output**
Type |Name| Description|
:---  | :--- |:--- | 
InliersCluster[]| cluster_objs| Clusters found

###primitive_services
4 different services are provided which perform the RANSAC algorithm in order to find 4 different shapes: 
-Sphere
-Cylinder
-Cone 
-Plane

**ROS parameters **
In each service the same parameters are needed.
Convention used 
Ros Name : NAME_PARAM_"SHAPE"_SRV_name
Default values: DEFAULT_PARAM_"SHAPE"_SRV_name
Variable name : input"Shape"SrvName
NameLaunchFile= "/pitt/srv/"shape"_segmentation/"name""
Each ros name and default value is defined in the service itself. 
Example 
Name: NormalDistanceWeight
Shape: Sphere
Ros Name: NAME_PARAM_SPHERE_SRV_NORMAL_DISTANCE_WEIGHT
Default Value : DEFAULT_PARAM_SPHERE_SRV_NORMAL_DISTANCE_WEIGHT
Variable Name: inputSphereSrvNormalDistanceWeight
Name Launch File: "/pitt/srv/sphere_segmentation/Normal_Distance_Weight"

 Name   |Default| Description|
---  |------ | --- |
NormalDistanceWeight|Sphere 0.001 Cone 0.0006 Cylinder 0.001 Plane 0.001|Defines the relative influence of the normals to the surfacedefines the relative influence ([0,1]) of the normals to the surface (within an angle of [o,pi/2]) to concur for model evaluation. |
DistanceThreshold|Sphere 0.007 Cone 0.0055 Cylinder 0.008 Plane 0.007|Defines how close a point must be to the model in order to be considered an inlier of the shape.|
MaxIteration|Sphere 1000 Cone 1000 Cylinder 1000 Plane 1000| Defines the maximum number of iteration of the RANSAC segmentation.|
MinRadiusLimit|Sphere 0.005 Cone 0.001 Cylinder 0.005 Plane --|Defines the minimum radius allowed. |
MaxRadiusLimit|Sphere 0.500 Cone 0.500 Cylinder 0.500 Plane --|Defines the maximum radius allowed. |
MinOpeningAngle|Sphere  --- Cone 10.0  Cylinder 50.0 Plane --|Defines the minimum opening angle (in degrees)|
MaxOpeningAngle|Sphere  ---Cone 170.0 Cylinder 180.0 Plane --|Defines the maximum opening angle (in degrees) |

 **Msg**
 **input**
Type |Name| Description|
:---|:---|:--- | 
sensor_msgs/PointCloud2 | cloud | Point Clouds that must be analyzed |
sensor_msgs/PointCloud2 | normals | Normals to the input point clouds |

**output**
Type |Name| Description|
:---  | :--- |:--- | 
int32[] | inliers |	Point belonging to the desired shape wrt the input cloud |
float32[] | coefficients |Coefficient of the found shape, they are different for different primitive shapes|
float32 | x_centroid | x centroid coordinate | 
float32 | y_centroid | y centroid coordiante |
float32 | z_centroid | z centroid coordinate |

