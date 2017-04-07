/**
 * Project@build<br>
 * /Project@build/[Source directory]/primitive_identification_tagging_and_tracking/pitt_object_table_segmentation/src/point_cloud_library/srv-manager.h<br>
 * <p>
 * Created on: Mar 7, 2016
 *     Author: luca-phd
 * Institution: University of Genoa, DIBRIS, EmaroLab
 * </p>
 * <p>
 * ...
 * </p>
 */

#ifndef SRV_MANAGER_H_
#define SRV_MANAGER_H_

#include <std_msgs/String.h>
#include "../point_cloud_library/pc_manager.h"

using namespace std;

	namespace srvm {

        // name of the services
        const string SRV_NAME_DEEP_FILTER = "deep_filter_srv";
        const string SRV_NAME_SUPPORT_FILTER = "support_segmentation_srv";
        const string SRV_NAME_CUSTER_FILTER = "cluster_Segmentation_srv";
        const string SRV_NAME_ARM_FILTER = "arm_filter_srv";


        // ROS parameters names
        const string NAME_PARAM_INPUT_CLOUD_REFERENCE_FRAME = "/pitt/ref_frame/input_cloud";
        const string NAME_PARAM_OUTPUT_CLOUD_REFERENCE_FRAME = "/pitt/ref_frame/output_cloud";
        const string NAME_PARAM_SRV_DEEP_FILTER_DEPTH_THRESHOLD = "/pitt/service/deep_filter/z_threshold";
        //ARM FILTER SERVICE
        const string NAME_PARAM_SRV_ARM_FILTER_FOREARM_MIN_BOX  = "/pitt/srv/arm_filter/min_forearm_box";
        const string NAME_PARAM_SRV_ARM_FILTER_FOREARM_MAX_BOX  = "/pitt/srv/arm_filter/max_forearm_box";
        const string NAME_PARAM_SRV_ARM_FILTER_ELBOW_MIN_BOX  = "/pitt/srv/arm_filter/min_elbow_box";
        const string NAME_PARAM_SRV_ARM_FILTER_ELBOW_MAX_BOX  = "/pitt/srv/arm_filter/max_elbow_box";
        // SUPPORT SEGMENTATION SERVICE
        const string NAME_PARAM_SRV_SUPPORT_MIN_ITERATIVE_CLOUD_PERCENTAGE = "/pitt/srv/supports_segmentation/min_iter_cloud_percent";
        const string NAME_PARAM_SRV_SUPPORT_MIN_ITERATIVE_SUPPORT_PERCENTAGE = "/pitt/srv/supports_segmentation/min_iter_support_percent";
        const string NAME_PARAM_SRV_SUPPORT_HORIZONTAL_VARIANCE_THRESHOLD = "/pitt/srv/supports_segmentation/horizontal_variance_th";
        const string NAME_PARAM_SRV_SUPPORT_RANSAC_IN_SHAPE_DISTANCE_POINT_THRESHOLD = "/pitt/srv/supports_segmentation/in_shape_distance_th";
        const string NAME_PARAM_SRV_SUPPORT_RANSAC_MODEL_NORMAL_DISTANCE_WEIGHT = "/pitt/srv/supports_segmentation/normal_distance_weight";
        const string NAME_PARAM_SRV_SUPPORT_RANSAC_MAX_ITERATION_THRESHOLD = "/pitt/srv/supports_segmentation/max_iter";
        const string NAME_PARAM_SRV_SUPPORT_HORIZONTAL_AXIS = "/pitt/srv/supports_segmentation/horizontal_axis";
        const string NAME_PARAM_SRV_SUPPORT_EDGE_REMOVE_OFFSET = "/pitt/srv/supports_segmentation/edge_remove_offset";




        //DEFAULT values
        //DEPTH FILTER
        const float DEFAULT_PARAM_SRV_DEEP_FILTER_DEPTH_THRESHOLD= 3.000f; // [m] [ baxter 2.61m]
        //ARM FILTER SERVICE
        const float MIN_FOREARM_BOX[] ={ -0.040f, -0.120f, -0.190f};
        const vector <float> DEFAULT_PARAM_SRV_ARM_FILTER_FOREARM_MIN_BOX(MIN_FOREARM_BOX,MIN_FOREARM_BOX+sizeof(MIN_FOREARM_BOX)/sizeof(float));
        const float MAX_FOREARM_BOX[] ={  0.340f,  0.120f,  0.105f};
        const vector <float> DEFAULT_PARAM_SRV_ARM_FILTER_FOREARM_MAX_BOX(MAX_FOREARM_BOX,MAX_FOREARM_BOX+sizeof(MAX_FOREARM_BOX)/sizeof(float));
        const float MIN_ELBOW_BOX[] = 	{ -0.090f, -0.135f, -0.160f};
        const vector <float> DEFAULT_PARAM_SRV_ARM_FILTER_ELBOW_MIN_BOX(MIN_ELBOW_BOX,MIN_ELBOW_BOX+sizeof(MIN_ELBOW_BOX)/sizeof(float));
        const float MAX_ELBOW_BOX[] ={  0.440f,  0.135f,  0.110f};
        const vector<float> DEFAULT_PARAM_SRV_ARM_FILTER_ELBOW_MAX_BOX(MAX_ELBOW_BOX,MAX_ELBOW_BOX+sizeof(MAX_ELBOW_BOX)/sizeof(float));
        //SUPPORT SEGMENTATION SERVICE
        const float DEFAULT_PARAM_SRV_SUPPORT_MIN_ITERATIVE_CLOUD_PERCENTAGE = 0.030f;
        const float DEFAULT_PARAM_SRV_SUPPORT_MIN_ITERATIVE_SUPPORT_PERCENTAGE = 0.030f;
        const float DEFAULT_PARAM_SRV_SUPPORT_HORIZONTAL_VARIANCE_THRESHOLD = 0.09f;
        const float DEFAULT_PARAM_SRV_SUPPORT_RANSAC_IN_SHAPE_DISTANCE_POINT_THRESHOLD = 0.02f;
        const float DEFAULT_PARAM_SRV_SUPPORT_RANSAC_MODEL_NORMAL_DISTANCE_WEIGHT = 0.9f; // [0,1]
        const int DEFAULT_PARAM_SRV_SUPPORT_RANSAC_MAX_ITERATION_THRESHOLD = 10;
        const float SUPPORT_HORIZZONTAL_AXIS[]={ 0.0f, 0.0f, -1.0f};
        const vector<float> DEFAULT_PARAM_SRV_SUPPORT_HORIZZONTAL_AXIS(SUPPORT_HORIZZONTAL_AXIS,SUPPORT_HORIZZONTAL_AXIS+sizeof(SUPPORT_HORIZZONTAL_AXIS)/sizeof(float));
        const float SUPPORT_EDGE_REMOVE_OFFSET[]={ 0.02, 0.02, 0.005};
        const vector<float> DEFAULT_PARAM_SRV_SUPPORT_EDGE_REMOVE_OFFSET(SUPPORT_EDGE_REMOVE_OFFSET,SUPPORT_EDGE_REMOVE_OFFSET+sizeof(SUPPORT_EDGE_REMOVE_OFFSET)/sizeof(float));  ; // in meters

        // parameter (not ros) default value
        //OBJECT SEGMENTATION NODE
        const string DEFAULT_PARAM_INPUT_CLOUD_REFERENCE_FRAME = "/camera_depth_optical_frame";
        const string DEFAULT_PARAM_OUTPUT_CLOUD_REFERENCE_FRAME = "/world";
        const string DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC = "/pepper_robot/camera/depth_registered/points"; 	// default for freenect driver
        const string DEFAULT_INPUT_PARAM_CENTROID_LOG_FILE_PATH = ""; 					// empty do not print
        const bool DEFAULT_INPUT_PARAM_SHOW_ORIGINAL_CLOUD = false; 				// with norms [white]
        const bool DEFAULT_INPUT_PARAM_SHOW_SUPPORT_CLOUDS = false;  						// [brown]
        const bool DEFAULT_INPUT_PARAM_SHOW_OBJECT_ON_SUPPORT = false; 				// as a unique cloud [orange]
        const bool DEFAULT_INPUT_PARAM_SHOW_CLUSTER_CLOUDS = false;						// as separate clusters [with random colors]
        //ARM FILTER SERVICE
        const string DEFAULT_PARAM_ARM_SRV_CAMERA_FRAME_NAME = "CameraDepth_optical_frame";
        const string DEFAULT_PARAM_ARM_SRV_RIGHT_FOREARM_FRAME_NAME = "RForeArm";
        const string DEFAULT_PARAM_ARM_SRV_LEFT_FOREARM_FRAME_NAME = "LForeArm";
        const string DEFAULT_PARAM_ARM_SRV_RIGHT_ELBOW_FRAME_NAME = "RElbow";
        const string DEFAULT_PARAM_ARM_SRV_LEFT_ELBOW_FRAME_NAME = "LElbow";
        const bool DEFAULT_PARAM_ARM_SRV_SHOW_CLOUDS = false;	// [red filtered points (arm)] [green remaining points]

        // topics (between nodes) names
        // TODO: ADJUST NAMES
        const string TOPIC_OUT_NAME_OBJECT_PERCEPTION = "obj_segmentation/ClusterOutput";
        const float DEFAULT_TF_WAIT_SECONDS = 2.0f;

        const string DEFAULT_SYMBOL = ".";

        string getStringParameter( string input, const string defaultValue){
            if( input == DEFAULT_SYMBOL)
                return defaultValue;
            return input;
        }
        string getStringPtrParameter( char *input, const string defaultValue){
            string inputStr( input);
            return getStringParameter( inputStr, defaultValue);
        }

        bool getBoolParameter( string input, const bool defaultValue){
            if( input == DEFAULT_SYMBOL)
                return defaultValue;
            return (bool) strtol( input.c_str(), NULL, 0);
        }
        bool getBoolPtrParameter( char *input, const bool defaultValue){
            string inputStr( input);
            return getBoolParameter( inputStr, defaultValue);
        }

        string getPathParameter( string input, const string defaultValue){
            if( input == DEFAULT_SYMBOL)
                return defaultValue;
            if( input.find( "..") != std::string::npos){
                string out = input.substr(0, input.size() - 2); // remove the ".."
                return out + pcm::PCManager::getFomrattedData();
            }
            return input;
        }
        string getPathPtrParameter( char *input, const string defaultValue){
            string inputStr( input);
            return getPathParameter( inputStr, defaultValue);
        }

        float getServiceFloatParameter( float input, const float defaultValue){
            if( input >= 0.0f)
                return input;
            return defaultValue;
        }
        int getServiceIntParameter( int input, const int defaultValue){
            if( input >= 0)
                return input;
            return defaultValue;
        }
        string getServiceStringParameter( string input, const string defaultValue){
            return getStringParameter( input, defaultValue);
        }
        vector<float> getService3DArrayParameter( vector<float> input, const vector<float> defaultValue){
            if( input.size() == 3)
                return input;
            return defaultValue;
        }
        vector< float> get3DArray( const float values[]){
            vector<float> vec( values, values + 3);
            return vec;
        }
        vector<float> getService3DArrayParameter( vector<float> input, const float defaultValue[]){
            vector<float> defaultVector = get3DArray( defaultValue);
            return getService3DArrayParameter( input, defaultVector);
        }


        string getFlagValueToPrint( bool flag){
            if( flag)
                return  "true  (1)";
            else return "false (0)";
        }
        string getArrayToPrint( vector< float> arr){
            string out = "[";
            if( arr.size() == 0)
                return out + "]";
            for( int i = 0; i < arr.size(); i++)
                if( i < arr.size() - 1)
                    out += boost::to_string( arr[ i]) + ", ";
                else out += boost::to_string( arr[ i]) + "]";
            return out;
        }
} /* namespace srvm */



#endif /* SRV_MANAGER_H_ */
