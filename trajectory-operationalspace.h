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

		class TrajectoryOperationCubic : public TrajectoryBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

				TrajectoryOperationCubic(const std::string & name);
			TrajectoryOperationCubic(const std::string & name, const Transform3d & init_M, const Transform3d & goal_M, const double & duration, const double & stime);

			unsigned int size() const;
			const TrajectorySample & operator()(double time);
			const TrajectorySample & computeNext();
			void getLastSample(TrajectorySample & sample) const;
			bool has_trajectory_ended() const;
			void setReference(Transform3d ref);
			void setInitSample(Transform3d init_M);
			void setGoalSample(Transform3d goal_M);
			void setDuration(const double & duration);
			void setCurrentTime(const double & time);
			void setStartTime(const double & time);


		protected:
			Transform3d m_ref;
			Transform3d m_init, m_goal, m_cubic;
			double m_duration, m_stime, m_time;


		};
	}
}


#endif // __HQP__TRAJECTOEIS__OPERATION__