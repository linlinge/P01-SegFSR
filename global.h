#pragma once
#include <iostream>
#include <string>
#include <vtkAutoInit.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "V2.hpp"
#include "V3.hpp"
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
typedef pcl::PointXYZRGBA PointType;

Eigen::Matrix3f Quaternion2Rotation(Eigen::Quaternionf q);
Eigen::Matrix3f Quaternion2Rotation(float qw,float qx,float qy,float qz);
void LoadPointSet(string filename, pcl::PointCloud<PointType>::Ptr cloud);
using namespace cv;