#include "global.h"
Eigen::Matrix3f Quaternion2Rotation(Eigen::Quaternionf q)
{
	Eigen::Matrix3f m;
	m(0,0)=pow(q.w(),2)+pow(q.x(),2)-pow(q.y(),2)-pow(q.z(),2);
	m(0,1)=2*(q.x()*q.y()-q.w()*q.z());
	m(0,2)=2*(q.x()*q.z()+q.w()*q.y());
	m(1,0)=2*(q.x()*q.y()+q.w()*q.z());
	m(1,1)=pow(q.w(),2)-pow(q.x(),2)+pow(q.y(),2)-pow(q.z(),2);
	m(1,2)=2*(q.y()*q.z()-q.w()*q.x());
	m(2,0)=2*(q.x()*q.z()-q.w()*q.y());
	m(2,1)=2*(q.y()*q.z()+q.w()*q.x());
	m(2,2)=pow(q.w(),2)-pow(q.x(),2)-pow(q.y(),2)+pow(q.z(),2);
	return m;
}

Eigen::Matrix3f Quaternion2Rotation(float qw,float qx,float qy,float qz)
{
	Eigen::Matrix3f m;
	m(0,0)=pow(qw,2)+pow(qx,2)-pow(qy,2)-pow(qz,2);
	m(0,1)=2*(qx*qy-qw*qz);
	m(0,2)=2*(qx*qz+qw*qy);
	m(1,0)=2*(qx*qy+qw*qz);
	m(1,1)=pow(qw,2)-pow(qx,2)+pow(qy,2)-pow(qz,2);
	m(1,2)=2*(qy*qz-qw*qx);
	m(2,0)=2*(qx*qz-qw*qy);
	m(2,1)=2*(qy*qz+qw*qx);
	m(2,2)=pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2);
	return m;
}

void LoadPointSet(string filename,pcl::PointCloud<PointType>::Ptr cloud)
{	
	string file_format=filename.substr(filename.length()-3);
	if(file_format=="ply")
	{
		if(pcl::io::loadPLYFile(filename,*cloud)==-1)
		{
			cout<<"Load PLY file failed!"<<endl;
		}
	}
	else if(file_format=="pcd")
	{
		if(pcl::io::loadPCDFile(filename, *cloud)==-1)
		{
			cout<<"Load PCD file failed!"<<endl;
		}
	}
}

float Norm(PointType p)
{
	return sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
}




Mat Vector2Rotation(V3 orientation_and_arc)
{	
	Mat rotation_vector = (Mat_<float>(3, 1) << orientation_and_arc.x,orientation_and_arc.y,orientation_and_arc.z);   // rotation vector
	Mat rotation_matrix;                                            // rotaiton matrix
	Rodrigues(rotation_vector, rotation_matrix);                    // calculate 
	return rotation_matrix;
}