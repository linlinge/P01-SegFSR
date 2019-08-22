#pragma once
#include <iostream>
#include <vector>
#include "global.h"
using namespace std;
#define IMAGE_WIDTH 800.0
class PixelInfo
{
	public:
		vector<PointType> dat_;		
};

class zBuffer
{
	public:
		int image_width_, image_height_;
		float actual_width_,actual_height_;
		float wstep_,hstep_;
		vector<vector<PixelInfo>> dat_;
		Mat img_;
		V2 v_left_up_,v_right_down_;
		pcl::PointCloud<PointType>::Ptr cloud_;		
		
		zBuffer(pcl::PointCloud<PointType>::Ptr cloud);
		void GetImage(int mode=0);
		void LoadMask(string mask_path);
};