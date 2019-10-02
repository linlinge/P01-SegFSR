#include "SegFSR.h"
ZBuffer::ZBuffer(int rows, pcl::PointCloud<PointType>::Ptr cloud,int axis)
{
	// Generate Picture
	PointType min,max;
    pcl::getMinMax3D(*cloud,min,max);
	float border_width=(max.x-min.x)*0.1;
	min.x=min.x-border_width;
	max.x=max.x+border_width;
	min.y=min.y-border_width;
	max.y=max.y+border_width;
	
	// initial
	rows_=rows; cols_=(int)((max.x-min.x)/(max.y-min.y)*rows_);
	vector<float> tmp;
	tmp.resize(cols_);
	for(int i=0;i<rows_;i++)
		depth_.push_back(tmp);
	
	for(int i=0;i<rows_;i++){
		for(int j=0;j<cols_;j++){
			depth_[i][j]=-INT_MAX;
		}
	}
	
	img_.create(rows_,cols_, CV_8UC3);
	for(int i=0;i<rows_;i++){
		for(int j=0;j<cols_;j++){
			img_.at<cv::Vec3b>(i,j)[0]=255;
			img_.at<cv::Vec3b>(i,j)[1]=255;
			img_.at<cv::Vec3b>(i,j)[2]=255;
		}
	}
	
	float delta_x=(max.x-min.x)/cols_;
	float delta_y=(max.y-min.y)/rows_;
	
	
	for(int k=0;k<cloud->points.size();k++)
	{
		int j=floor((cloud->points[k].x-min.x)/delta_x);
		int i=floor((cloud->points[k].y-min.y)/delta_y);
		//cout<<"("<<i<<","<<j<<",["<<cloud->points[k].x<<","<<cloud->points[k].y<<","<<cloud->points[k].z<<"])"<<endl;
		
		if(depth_[i][j]<cloud->points[k].z){
			depth_[i][j]=cloud->points[k].z;
			
			
			/* 
			img_.at<Vec3b>(i,j)[0]=cloud->points[k].b;
			img_.at<Vec3b>(i,j)[1]=cloud->points[k].g;
			img_.at<Vec3b>(i,j)[2]=cloud->points[k].r; 
			*/
			
			img_.at<cv::Vec3b>(i,j)[0]=0;
			img_.at<cv::Vec3b>(i,j)[1]=0;
			img_.at<cv::Vec3b>(i,j)[2]=0;
		}
	}

	
	
	cv::imshow("3D Viewer",img_);
	cv::imwrite("1.jpg",img_);
	cv::waitKey(0);
}


SegFSR::SegFSR(pcl::PointCloud<PointType>::Ptr cloud, float delta_arc,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	cloud_=cloud;
	delta_arc_=delta_arc;
	viewer_=viewer;
	viewer_->setBackgroundColor(1.0, 1.0, 1.0);
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
	cv::Mat mat_initial_vector = (cv::Mat_<double>(3, 1) << v_forward_.x, v_forward_.y , v_forward_.z);   // initial vector
	int n=floor(2*CV_PI/delta_arc_);
	for(int i=0;i<=n;i++)
	{
		V3 v3_rotation_vector= v_upright_*delta_arc_*i;	
		cv::Mat mat_rotation_vector = (cv::Mat_<double>(3, 1) << v3_rotation_vector.x, v3_rotation_vector.y, v3_rotation_vector.z); 		
		cv::Mat rotation_matrix;                                            // rotaiton matrix
		Rodrigues(mat_rotation_vector, rotation_matrix);
		cv::Mat des_vector = rotation_matrix * mat_initial_vector;
		
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
	
	// 
	Projection(orientations_[0],400);
	
	
	// Find the outliers
	/* for(int k=0; k< orientations_.size();k++){		
		// Generate projection images
		Projection(orientations_[k]);
	
		// Image Segmentation
		
		
		
		// Store outlier indices
	
		break;
	} */
	
	// Remova all outliers 		
	
	
	// demonstrate the result
	/* cout<<"cloud_"<<endl;
	Viewer(cloud_,"cloud"); */
}


cv::Mat SegFSR::Projection(V3 projection_orientation,int rows)
{	
	pcl::PointCloud<PointType>::Ptr tf_cloud (new pcl::PointCloud<PointType> ());
	
	
	// define affine
	float alpha=projection_orientation.GetArcToPlane(Z_AXIS,YOZ);
	float beta=projection_orientation.GetArcToPlane(X_AXIS,XOZ);
	
	Eigen::Affine3f tf = Eigen::Affine3f::Identity();
	tf.translation()<<0,0,0;
	tf.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()));
	tf.rotate(Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitX()));	
	pcl::transformPointCloud (*cloud_, *tf_cloud, tf);	
	ZBuffer zb(rows,tf_cloud,Z_AXIS);			
	return zb.img_;	
}
 


void SegFSR::Viewer(pcl::PointCloud<PointType>::Ptr cloud,string id)
{
	
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud); 	
	viewer_->addPointCloud<PointType> (cloud, multi_color, id);  
	
/* 	// add arrow
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
	} */
	
}