#ifndef __HQP__TRAJECTOEIS__BASE__
#define __HQP__TRAJECTOEIS__BASE__

#include "fwd.h"

#include <string>

namespace HQP {
	namespace trajectories {

		class TrajectorySample
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			VectorXd pos, vel, acc;

			TrajectorySample(unsigned int size = 0)
			{
				resize(size, size);
			}

			TrajectorySample(unsigned int size_pos, unsigned int size_vel)
			{
				resize(size_pos, size_vel);
			}

			void resize(unsigned int size)
			{
				resize(size, size);
			}

			void resize(unsigned int size_pos, unsigned int size_vel)
			{
				pos.setZero(size_pos);
				vel.setZero(size_vel);
				acc.setZero(size_vel);
			}
		};


		class TrajectoryBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				TrajectoryBase(const std::string & name) : m_name(name) {}

			virtual unsigned int size() const = 0;
			virtual const TrajectorySample & operator()(double time) = 0;
			virtual const TrajectorySample & computeNext() = 0;
			virtual const TrajectorySample & getLastSample() const { return m_sample; }
			virtual void getLastSample(TrajectorySample & sample) const = 0;
			virtual bool has_trajectory_ended() const = 0;

		protected:
			std::string m_name;
			TrajectorySample m_sample;
		};
	}
}


#endif // __HQP__TRAJECTOEIS__BASE__