#ifndef PCMANAGER_H_
#define PCMANAGER_H_

#include <pcl_ros/point_cloud.h>				// point cloud library
#include <pcl/point_types.h>					// for normal estimation
#include <pcl/features/normal_3d.h>				// for normal estimation
#include <pcl/filters/voxel_grid.h> 			// for dowsampling
#include <pcl/visualization/pcl_visualizer.h> 	// for pcl visualizer
#include <std_msgs/String.h>					// for PointCloud2
#include <math.h>								// for ysing random function

#include "../point_cloud_library/pc_primitive.h"			 			// for shape custom class


// useful class name used
using namespace pcl;
using namespace std;
using namespace pcp;
using namespace sensor_msgs;

// nice abbreviation for pointer to custom class
typedef boost::shared_ptr< pcp::PCPrimitive> PCPrimitivePtr;			// for the list of the primitives
typedef boost::shared_ptr< visualization::PCLVisualizer> PCLVisualizer;	// for visualization window

namespace pcm {

	class PCManager {
		private:
			PCLCloudPtr originalCloud;		 // contains the cloud from which all the other clouds depends on
			PCLNormalPtr originalNorms;	 // contains the normal vector of the original cloud

			PCLVisualizer visor;   		 // variable to visualize point cloud
			bool visualizationFlag;		 // variable to show or not visualization window

			vector< PCPrimitivePtr> primitiveList;	// contains all the detached primitive shapes

			void initialize( bool visualizationFlag);	// common initialization method for all constructors

		public:
			// constructor
			PCManager();
			PCManager( bool visualizationFlag);
			// deconstructor
			virtual ~PCManager();

			// static method to copy entities
			static PCLCloudPtr copyCloud( PCLCloudPtr input);
			static PCLNormalPtr copyNormals( PCLNormalPtr input);
			static ModelCoefficients::Ptr copyCoefficients( ModelCoefficients::Ptr input);
			// static methods for common preprocessing
			static PCLCloudPtr downSampling( PCLCloudPtr input); // with default parameters
			static PCLCloudPtr downSampling( PCLCloudPtr input, float span);
			static PCLCloudPtr downSampling( PCLCloudPtr input, float spanX, float spanY, float spanZ);
			static PCLNormalPtr estimateNormal( PCLCloudPtr input); // with default parameter
			static PCLNormalPtr estimateNormal( PCLCloudPtr input, int search);
			// to convert ros point cloud to pcl point cloud and vice-versa
			static PointCloud2 cloudToRosMsg( PCLCloudPtr input);
			static PCLCloudPtr cloudForRosMsg( PointCloud2 input);
			static PCLCloudPtr cloudForRosMsg( PointCloud2Ptr input);
			static PointCloud2 normToRosMsg( PCLNormalPtr input);
			static PCLNormalPtr normForRosMsg( PointCloud2 input);
			static vector< int> inlierToVectorMsg( PointIndices::Ptr inliers);
			static vector< float> coefficientToVectorMsg( ModelCoefficients::Ptr coefficients);
			// methods for visualization
			static PCLVisualizer createVisor( string title); // method to create visualization window
			static void updateVisor ( PCLVisualizer viewer, PCLCloudPtr cloud, int R, int G, int B, string name);// method to show a new cloud
			static void updateVisor ( PCLVisualizer viewer, PCLCloudPtr cloud, string name); // method to show a new cloud (with random color)
			static void updateVisor ( PCLVisualizer viewer, PCLCloudPtr cloud, PCLNormalPtr normals, int R, int G, int B, string name); // method to show a new cloud with normals
			static void updateVisor ( PCLVisualizer viewer, PCLCloudPtr cloud, PCLNormalPtr normals, string name); // method to show a new cloud with normals with random color
			static void updateVisor ( PCLVisualizer viewer, PointXYZRGB point, int R, int G, int B, string name); // show a single point
			static void updateVisor ( PCLVisualizer viewer, PointXYZRGB point, string name); // show a single point (with random color))
			static void clearVisor( PCLVisualizer viewer);
			// get cloud from original cloud and inlier idx (for debug)
			static vector< PCLCloudPtr> getCloudFromIdx( PCLCloudPtr originalCloud, PrimitiveIdxPtr indices);
			//static PCLCloudPtr getCloudFromIdx( PCLCloudPtr originalCloud, PrimitiveIdxPtr indices, int level); // level < -1

			static string getFomrattedData();

			// getCloudFromIdx usign the original cloud instanciate in this object
			vector< PCLCloudPtr> getCloudFromIdx( PrimitiveIdxPtr indices);
			//vector< PCLCloudPtr> getCloudFromIdx( PrimitiveIdxPtr indices, int level);

			// visualize this object
			void visualize(); // visualize original cloud and norms (if they exist)

			// methods for primitive list
			PCPrimitivePtr getPrimitiveShape( int idx);
			int addPrimitiveShape( string shapeName, PCLCloudPtr cloud, PCLNormalPtr norms, bool visualFlag); // returns the new idx
			int clearPtimitiveShape(); // returns the number of item deleted

			// getter methods
			PCLCloudPtr getOriginalCloud();
			PointCloud2 getOriginalCloudRosMsg();
			PCLNormalPtr getOriginalNormal(); // computed during setOriginalCloud()
			PointCloud2 getOriginalNormalRosMsg();
			bool getVisualizationFlag();
			PCLVisualizer getVisor();

			// setter methods
			void setOriginalCloud( PCLCloudPtr cloud);
			void setOriginalCloud( PCLCloudPtr cloud, int normSearch, float downSpanX, float downSpanY, float downSpanZ);
			void setOriginalCloud( PointCloud2Ptr cloud);
			void setOriginalCloud( PointCloud2Ptr cloud, int normSearch, float downSpanX, float downSpanY, float downSpanZ);
			void setVisualizationFlag( bool flag); // open new viewer if comes from false to true

			static bool writeToFile( string txt, string filePath, bool append);

			// constants for visualization
			static const bool DEFAULT_VISUALIZATION_FLAG;
			static const int VISUALIZER_POINT_SIZE;
			static const int VISUALIZER_POINT_SIZE_BIG;
			static const string DEFAULT_CLOUD_NAME_SUFFIX;
			static const string DEFAULT_NORM_NAME_SUFFIX;
			static const string DEFAULT_ORIGINAL_CLOUD_VIEWER_NAME;
			static const int DEFAULT_NORM_LEVEL;
			static const float DEFAULT_NORM_SCALE ;
			static const string DEFAULT_VISUALIZER_TITLE;
			// for preprocessing
			static const int DEFAULT_NORM_SEARCH;// = 20;
			static const float DEFAULT_DOWSEAMPLIG_RATE;// =  0.008;

	};

} /* namespace pcm */

#endif /* PCMANAGER_H_ */
