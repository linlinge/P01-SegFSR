#include "global.h"
#include "BoundingBox.h"
/*
	Image Segmentation based 2D-3D Fusion for 3D Object Filtering, Segmentation and Recognition
*/
class ZBuffer
{
	public:
		int rows_,cols_;
		vector<vector<float>> depth_;
		Mat img_;
		ZBuffer(int rows, pcl::PointCloud<PointType>::Ptr cloud, int axis);
};

class SegFSR
{
	public:
		vector<V3> orientations_;
		vector<int> outliers_idx_;  	// store the indices for outliers
		pcl::PointCloud<PointType>::Ptr cloud_;
		float delta_arc_;
		PointType p_upright_,p_forward_,p_left_,p_centre_;
		V3 v_upright_,v_forward_,v_left_;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
		
		
		SegFSR(pcl::PointCloud<PointType>::Ptr cloud, float delta_arc,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);  // initial
		void UprightEstimation();
		void OrientationsGenerator();
		Mat Projection(V3 projection_orientation,int rows);
		void Viewer(pcl::PointCloud<PointType>::Ptr cloud,string id);
		void Run();		
};











