#include "SegFSR.h"
void Orientations::DirectionGenerator(V3 centre, V3 upright,V3 initial_vector, float delta_arc)
{	
	V3 v3_left=Cross(initial_vector,upright);
	initial_vector=Cross(upright,v3_left);
	
	
	// store parameters
	upright_=upright;
	initial_vector_=initial_vector;
	

	Mat mat_initial_vector = (Mat_<double>(3, 1) << initial_vector.x, initial_vector.y , initial_vector.z);   // initial vector
	int n=floor(2*CV_PI/delta_arc);
	for(int i=0;i<=n;i++)
	{
		V3 v3_rotation_vector= upright.Normalize()*delta_arc*i;	
		Mat mat_rotation_vector = (Mat_<double>(3, 1) << v3_rotation_vector.x, v3_rotation_vector.y, v3_rotation_vector.z); 		
		Mat rotation_matrix;                                            // rotaiton matrix
		Rodrigues(mat_rotation_vector, rotation_matrix);
		Mat des_vector = rotation_matrix * mat_initial_vector;
		
		V3 tmp=V3(des_vector.ptr  <double>(0)[0],des_vector.ptr<double>(1)[0],des_vector.ptr<double>(2)[0]);
		tmp=tmp.Normalize();
		orientations_.push_back(tmp);
	}
}