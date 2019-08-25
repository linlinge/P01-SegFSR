#include "SegFSR.h"
SegFSR::SegFSR(pcl::PointCloud<PointType>::Ptr cloud, float delta_arc)
{
	cloud_=cloud;
	delta_arc_=delta_arc;
}

void SegFSR::UprightEstimation()
{
	BoundingBox bb(cloud_);
	
	p_upright_=bb.pcZ_;
	p_forward_=bb.pcX_;
	p_left_=bb.pcY_;
	p_centre_=bb.cp_;
	
	v_upright_=V3(p_upright_.x-p_centre_.x,p_upright_.y-p_centre_.y,p_upright_.z-p_centre_.z);
	v_forward_=V3(p_forward_.x-p_centre_.x,p_forward_.y-p_centre_.y, p_forward_.z-p_centre_.z);
	v_left_=V3(p_left_.x-p_centre_.x,p_left_.y-p_centre_.y,p_left_.z-p_centre_.z);
	
	v_upright_=v_upright_/v_upright_.Norm();
	v_forward_=v_forward_/v_forward_.Norm();
	v_left_=v_left_/v_left_.Norm();
}


void SegFSR::OrientationsGenerator()
{		
	Mat mat_initial_vector = (Mat_<double>(3, 1) << v_forward_.x, v_forward_.y , v_forward_.z);   // initial vector
	int n=floor(2*CV_PI/delta_arc_);
	for(int i=0;i<=n;i++)
	{
		V3 v3_rotation_vector= v_upright_*delta_arc_*i;	
		Mat mat_rotation_vector = (Mat_<double>(3, 1) << v3_rotation_vector.x, v3_rotation_vector.y, v3_rotation_vector.z); 		
		Mat rotation_matrix;                                            // rotaiton matrix
		Rodrigues(mat_rotation_vector, rotation_matrix);
		Mat des_vector = rotation_matrix * mat_initial_vector;
		
		V3 tmp=V3(des_vector.ptr  <double>(0)[0],des_vector.ptr<double>(1)[0],des_vector.ptr<double>(2)[0]);
		tmp=tmp.Normalize();
		orientations_.push_back(tmp);
	}	
}



void SegFSR::Run()
{
	// upright estimation 
	UprightEstimation();
	
	// Generate projection orientations
	OrientationsGenerator();
	
	// Find the outliers
	/* for(int k=0; k< orientations_.size();k++){		
		// Generate projection images
		Projection(orientations_[k]);
	
		// Image Segmentation
		
		
		
		// Store outlier indices
	
		break;
	} */
	
	// Remova all outliers 	
	
	Viewer(cloud_);
}


Mat SegFSR::Projection(V3 projection_orientation)
{
/* 	pcl::PointCloud<PointType>::Ptr cloud_f(new pcl::PointCloud<PointType>);
	
	//我们使用平面模型，其中ax + by + cz + d = 0，其中a = b = d = 0，并且c = 1，或者换句话说，XY平面。
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 0;
	coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	//我们创建投影对象，并使用上面定义的模型作为投影模型
	pcl::ProjectInliers<PointType> prj;
	prj.setModelType(pcl::SACMODEL_PLANE);
	prj.setInputCloud(cloud_);
	prj.setModelCoefficients(coefficients);
	prj.filter(*cloud_f);
	
	Viewer(cloud_f); */
}
 

void SegFSR::Viewer(pcl::PointCloud<PointType>::Ptr cloud)
{
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud); 	
	viewer.addPointCloud<PointType> (cloud, multi_color, "sample cloud");  
	
	// add arrow
	viewer.addArrow<PointType>(p_upright_, p_centre_, 1.0f, 0, 0, false, "X");
	viewer.addArrow<PointType>(p_forward_, p_centre_, 0.0f, 1.0f, 0, false, "Y");
	viewer.addArrow<PointType>(p_left_, p_centre_, 0.0f, 0.0f, 1.0f, false, "Z");
	
	
	for(int i=0;i< orientations_.size();i++)
	{
		string tmp="orientation"+std::to_string(i);
		PointType p_tmp;
		p_tmp.x=orientations_[i].x+p_centre_.x;
		p_tmp.y=orientations_[i].y+p_centre_.y;
		p_tmp.z=orientations_[i].z+p_centre_.z;
		viewer.addArrow<PointType>(p_tmp, p_centre_, 1.0f, 1.0f, 0.0f, false, tmp);
	}
	
	while(!viewer.wasStopped()){	
		viewer.spin();
		boost::this_thread::sleep (boost::posix_time::microseconds (10));
	}
}