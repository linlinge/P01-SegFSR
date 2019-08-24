#include "global.h"
#include "BoundingBox.h"
/*
	Image Segmentation based 2D-3D Fusion for 3D Object Filtering, Segmentation and Recognition
*/
class SegFSR
{
	public:
		vector<V3> orientations_;
		vector<int> outliers_idx_;  	// store the indices for outliers
		pcl::PointCloud<PointType>::Ptr cloud_;
		float delta_arc_;
		PointType p_upright_,p_forward_,p_left_,p_centre_;
		V3 v_upright_,v_forward_,v_left_;
		pcl::visualization::PCLVisualizer viewer;
		
		
		SegFSR(pcl::PointCloud<PointType>::Ptr cloud, float delta_arc);  // initial
		void UprightEstimation();
		void OrientationsGenerator();
		Mat Projection(V3 projection_orientation);
		void Viewer(pcl::PointCloud<PointType>::Ptr cloud);
		void Run();
};











