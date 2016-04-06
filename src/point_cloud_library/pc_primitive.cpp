#include "../point_cloud_library/pc_primitive.h"

namespace pcp {

// ######################################## CONSTANTS ##################################################

	// for the shape name
	const string PCPrimitive::DEFAULT_SHAPE_NAME_PLANE = "plane";
	const string PCPrimitive::DEFAULT_SHAPE_NAME_CLUSTER = "cluster";
	// for visualization w.r.t to index
	const string PCPrimitive::DEFAULT_VISUALIZATION_NAME_SEPARATOR = "-";

// ######################################## GLOBAL VARAIBLE ############################################


// ########################################## PUBLIC ###################################################
	PCPrimitive::PCPrimitive( string shapename, int shapeMapidx, bool visualFlag, PCLCloudPtr cloud, PCLNormalPtr norms){
		shapeName = shapename;
		visualizationName = getVisualizationNameFromTag( shapeMapidx);
		shapeMapIdx = shapeMapidx;
		visualizationFlag = visualFlag;
		primitiveCloud = *cloud;//primitiveCloud = copyCloud( cloud);
		primitiveNormals = *norms;//primitiveNormals = copyNormals( norms);
	}

	PCPrimitive::~PCPrimitive() {
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}

// ########################################## PRIVATE ###################################################

	// initialize the visualization name w.r.t. the shape name and an index
	string PCPrimitive::getVisualizationNameFromTag( int idx){
		return( getShapeName() + DEFAULT_VISUALIZATION_NAME_SEPARATOR + boost::to_string( idx));
	}

	//PrimitiveIdx PCPrimitive::copyVector( PrimitiveIdxPtr input){
	//	PrimitiveIdxPtr output ( new PrimitiveIdx);
	//	for( int i = 0; i < input->size(); i++)
	//		output->push_back( (* input)[ i]);
	//	return( *output);
	//}
	/*PointCloud< PointXYZ> PCPrimitive::copyCloud( PointCloud< PointXYZ>::Ptr input){
		// create cloud
		PointCloud< PointXYZ>::Ptr output ( new PointCloud< PointXYZ>);
		// copy it
		for( int i = 0; i < input->size(); i++){
			// remove Nan
			//if( input->points[ i].x == input->points[ i].x && input->points[ i].y == input->points[ i].y && input->points[ i].z == input->points[ i].z){
				PointXYZ *p ( new PointXYZ( input->points[ i].x, input->points[ i].y, input->points[ i].z));
				output->push_back( *p);
			//}
		}
		return( *output);
	}
	PointCloud< Normal> PCPrimitive::copyNormals( PointCloud< Normal>::Ptr input){
		PointCloud< Normal>::Ptr output (new PointCloud< Normal>);
		for( int i = 0; i < input->size(); i++)
			output->push_back( input->points[ i]);
		return( *output);
	}*/
	ModelCoefficients PCPrimitive::copyCoefficients( ModelCoefficients::Ptr input){
		ModelCoefficients::Ptr output ( new ModelCoefficients);
		for( int i = 0; i < input->values.size(); i++)
			output->values.push_back( input->values[ i]);
		return( *output);
	}


	/////////////////////////////// GETTER METHODS
	string PCPrimitive::getShapeName(){
		return( shapeName);
	}
	string PCPrimitive::getVisualizationName(){
		return( visualizationName);
	}
	bool PCPrimitive::getVisualizationFlag(){
		return( visualizationFlag);
	}
	int PCPrimitive::getShapeMapidx(){
		return( shapeMapIdx);
	}
	//PrimitiveIdx PCPrimitive::getPrimitiveIdx{

	//}
	PCLCloud PCPrimitive::getPrimitiveCloud(){
		return( primitiveCloud);
	}
	PCLNormal PCPrimitive::getPrimitiveNormal(){
		return( primitiveNormals);
	}
	//ModelCoefficients PCPrimitive::getPrimitiveCoefficients(){

	//}
	//string PCPrimitive::getPrimitiveCoefficientTxt(){

	//}
	//string PCPrimitive::getPrimitiveIdxTxt(){

	//}



} /* namespace pcp */
