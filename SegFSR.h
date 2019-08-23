#include "global.h"
/*
		Generate projection orientations
*/
class Orientations
{
	public:		
		vector<V3> dat_;
		V3 upright_;
		V3 initial_vector_;
		
		void Generator(V3 centre, V3 upright,V3 ref, float delta_arc);
};

/*
		Segmentation based 2D-3D Fusion for 3D Filtering, Segmentation and Recognition
*/
class SegFSR
{
	public:
		Orientations orientations_;		// store the orientations
		vector<int> outliers_idx_;  	// store the indices for outliers
		
		void Init(V3 centre, V3 upright,V3 ref, float delta_arc);
		
};











