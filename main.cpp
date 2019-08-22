#include "global.h"
#include "FileExtend.h"
#include "BoundingBox.h"
#include "BasicGeometry.h"
#include "zBuffer.h"
#include "V3.hpp"
#include <pcl/registration/transformation_estimation_3point.h>
using namespace std;
using namespace cv;
using namespace pcl;


int main(int argc, char **argv)
{	
	// load point cloud 	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	LoadPointSet(argv[1],cloud);
	
	
	
	
	
	/* // get bounding box
	BoundingBox bb(cloud,"cloud");	
	
	// projection
	Line line2(V3(bb.v1_.x,bb.v1_.y,bb.v1_.z),V3(bb.v2_.x,bb.v2_.y,bb.v2_.z),PP);
	V3 x2_prime=line2.TransformTo(XY);	
	Line line3(V3(bb.v1_.x,bb.v1_.y,bb.v1_.z),V3(bb.v3_.x,bb.v3_.y,bb.v3_.z),PP);
	V3 x3_prime=line3.TransformTo(XY);	
	Line line4(V3(bb.v1_.x,bb.v1_.y,bb.v1_.z),V3(bb.v4_.x,bb.v4_.y,bb.v4_.z),PP);
	V3 x4_prime=line4.TransformTo(XY);
	
	
	// get rigid transformation
	pcl::registration::TransformationEstimation3Point<PointXYZ, PointXYZ> te;
    pcl::registration::TransformationEstimation3Point<PointXYZ, PointXYZ>::Matrix4 R;
    pcl::PointCloud<PointXYZ> src, tgt;
    src.resize(3); tgt.resize(3);
    src.points[0].getVector3fMap() << bb.v2_.x, bb.v2_.y, bb.v2_.z;
    src.points[1].getVector3fMap() << bb.v3_.x, bb.v3_.y, bb.v3_.z;
    src.points[2].getVector3fMap() << bb.v4_.x, bb.v4_.y, bb.v4_.z;
    // std::cout << "source cloud is " << endl << src.getMatrixXfMap(3, 4, 0) << endl;
    tgt.points[0].getVector3fMap() << x2_prime.x,x2_prime.y,x2_prime.z;
    tgt.points[1].getVector3fMap() << x3_prime.x,x3_prime.y,x3_prime.z;
    tgt.points[2].getVector3fMap() << x4_prime.x,x4_prime.y,x4_prime.z;
    // std::cout << "target cloud is " << endl << tgt.getMatrixXfMap(3, 4, 0) << endl;
    te.estimateRigidTransformation(src, tgt, R);
    // std::cout << "computed transformation is \n" << R << endl;	
	
	// transform point cloud
	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());
	pcl::transformPointCloud(*cloud, *transformedCloud, R);
			
	
	// project to image plane	
	zBuffer buf(transformedCloud);	
	buf.GetImage(0);
	imwrite(string(argv[2]),buf.img_);
	buf.LoadMask("1-mask.png");  */
	
	

	
	// display point cloud
	pcl::visualization::PCLVisualizer viewer;
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);  //输入的初始点云相关
	viewer.addPointCloud(cloud, multi_color, "cloud");	
	DisplayBoundingBox(viewer,cloud,BoundingBox(cloud,"cloud",V3(0.0,1.0,1.0))); 

	/* pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color2(transformedCloud);  //输入的初始点云相关
	viewer.addPointCloud(transformedCloud, multi_color2, "cloud2");	
	DisplayBoundingBox(viewer,transformedCloud,BoundingBox(transformedCloud,"transformedCloud",V3(1.0,0.0,0.0)));  */
																
	// display 	
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "bbox");	
	//viewer.addCoordinateSystem(0.5f);
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
	
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}  
	

	
	// projection
	/* 
		Mat img(IMG_SCALE, IMG_SCALE*ratio_width_height,CV_8UC3,Scalar(100,100,0));
		imshow("Image Viewer",img);
		waitKey(0); 
	*/
	
	return 0;
} 


