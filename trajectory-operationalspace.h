#ifndef __HQP__TRAJECTOEIS__OPERATION__
#define __HQP__TRAJECTOEIS__OPERATION__

#include "trajectory-base.h"

#include <string>

namespace HQP {
	namespace trajectories {
		class TrajectoryOperationConstant : public TrajectoryBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			TrajectoryOperationConstant(const std::string & name);
			TrajectoryOperationConstant(const std::string & name, const Transform3d & M);

			unsigned int size() const;
			void setReference(const Transform3d ref);
			const TrajectorySample & operator()(double time);
			const TrajectorySample & computeNext();
			void getLastSample(TrajectorySample & sample) const;
			bool has_trajectory_ended() const;

		protected:
			Transform3d m_ref;
		};
	}
}


#endif // __HQP__TRAJECTOEIS__OPERATION__