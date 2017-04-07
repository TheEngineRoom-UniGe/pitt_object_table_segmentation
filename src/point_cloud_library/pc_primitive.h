
#ifndef PCPRIMITIVE_H_
#define PCPRIMITIVE_H_

#include <vector> 					// for array
#include <pcl_ros/point_cloud.h>	// for pcl
#include <std_msgs/String.h>		// for PointCloud2
// cusom class
//#include "PCManager.h"

// useful name space
using namespace std;
using namespace pcl;
using namespace sensor_msgs;
//using namespace pcm;

// nice abbreviation for pointer to custom class
typedef std::vector< int> PrimitiveIdx;									// for refer cloud points to original cloud index
typedef boost::shared_ptr< std::vector< int> > PrimitiveIdxPtr;			// for refer cloud points to original cloud index smart pointer
typedef pcl::PointCloud< pcl::PointXYZRGB> PCLCloud;						// for point cloud
typedef pcl::PointCloud< pcl::PointXYZRGB>::Ptr PCLCloudPtr;				// for point cloud smart pointer
typedef pcl::PointCloud< pcl::Normal> PCLNormal;						// for normal estimation of a point cloud
typedef pcl::PointCloud< pcl::Normal>::Ptr PCLNormalPtr;				// for normal estimation of a point cloud smart pointer

namespace pcp {

	class PCPrimitive {
		private:
			// common attributes for all the primitive shapes
			string shapeName;	 // to discriminate {PLANE, SPHERE, CYLINDER, CONE}
			string visualizationName; // visualizer needs a unique string tag for all visualised shape
			bool visualizationFlag;		// visualize or not from the main viewer of the PCManager

			int shapeMapIdx; // the index of the shape map in which this primitive is stored from the PCManager

			//PrimitiveIdx primitiveIdx; // w.r.t originalCloud contains {0:outliers, 1:inliers} (PRIMITIVE_IDX_... constants)
			PCLCloud primitiveCloud; // inlier point cloud
			PCLNormal primitiveNormals; // inlier normal estimation
			ModelCoefficients primitiveCoefficients; // shape coefficient (perhaps given by RANSAC)

			//string primitiveCoefficientTxt; // describe shape coefficients for glob fit algorithm
			//string primitiveIdxTxt;	// describe shape idx w.r.t. original cloud for glob fit algorithm

			// set the visualization name from shape name and an index
			string getVisualizationNameFromTag( int idx);

			// to correctly initialize variables
			//PrimitiveIdx PCPrimitive::copyVector( PrimitiveIdxPtr input);
			//PointCloud< PointXYZ> copyCloud( PointCloud< PointXYZ>::Ptr input);
			//PointCloud< Normal> copyNormals( PointCloud< Normal>::Ptr input);
			ModelCoefficients copyCoefficients( ModelCoefficients::Ptr input);

		public:
			// constructor
			PCPrimitive( string shapename, int shapeMapidx, bool visualFlag, PCLCloudPtr cloud, PCLNormalPtr norms);

			// deconstructor
			virtual ~PCPrimitive();

			// getter methods
			string getShapeName();
			string getVisualizationName();
			bool getVisualizationFlag();
			int getShapeMapidx();
			//PrimitiveIdx getPrimitiveIdx;
			PCLCloud getPrimitiveCloud();
			PCLNormal getPrimitiveNormal();
			//ModelCoefficients getPrimitiveCoefficients();
			//string getPrimitiveCoefficientTxt();
			//string getPrimitiveIdxTxt();

			// for shape name
			static const string DEFAULT_SHAPE_NAME_PLANE;
			static const string DEFAULT_SHAPE_NAME_CLUSTER;
			// for visualization name w.r.t index
			static const string DEFAULT_VISUALIZATION_NAME_SEPARATOR;
	};

} /* namespace pcp */

#endif /* PCPRIMITIVE_H_ */
