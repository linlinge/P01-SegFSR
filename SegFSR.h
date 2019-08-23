#include "global.h"
class ProjectionOrientation
{
	public:		
		vector<V3> orientations_;
		V3 upright_;
		V3 initial_vector_;
		
		void DirectionGenerator(V3 centre, V3 upright,V3 ref, float delta_arc);
};
