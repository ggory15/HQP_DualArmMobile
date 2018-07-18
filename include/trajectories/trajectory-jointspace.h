#ifndef __HQP__TRAJECTOEIS__JOINT__
#define __HQP__TRAJECTOEIS__JOINT__

#include "trajectory-base.h"

#include <string>

namespace HQP {
	namespace trajectories {
		class TrajectoryJointConstant : public TrajectoryBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			TrajectoryJointConstant(const std::string & name);
			TrajectoryJointConstant(const std::string & name, Cref_vectorXd ref);

			unsigned int size() const;
			void setReference(Cref_vectorXd ref);
			const TrajectorySample & operator()(double time);
			const TrajectorySample & computeNext();
			void getLastSample(TrajectorySample & sample) const;
			bool has_trajectory_ended() const;

		protected:
			VectorXd m_ref;
		};
	}
}


#endif // __HQP__TRAJECTOEIS__JOINT__