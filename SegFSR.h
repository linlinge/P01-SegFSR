#include "BoundingBox.h"
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "V2.hpp"
#include "V3.hpp"
#include "BasicGeometry.h"

/*
	Image Segmentation based 2D-3D Fusion for 3D Object Filtering, Segmentation and Recognition
*/
class ZBuffer
{
	public:
		int rows_,cols_;
		vector<vector<float>> depth_;
		cv::Mat img_;
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
		cv::Mat Projection(V3 projection_orientation,int rows);
		void Viewer(pcl::PointCloud<PointType>::Ptr cloud,string id);
		void Run();		
};











