#ifndef __HQP__Solver_FWD__
#define __HQP__Solver_FWD__

#include "fwd.h"
#include "constraint/fwd_constraints.h"
#include "utils/container.h"

using namespace HQP::container;

#define DEFAULT_HESSIAN_REGULARIZATION 1e-8
namespace HQP {
	namespace solver {
		enum SolverHQP
		{
			SOLVER_HQP_EIQUADPROG = 0,
			SOLVER_HQP_EIQUADPROG_FAST = 1,
			SOLVER_HQP_EIQUADPROG_RT = 2,
			SOLVER_HQP_QPOASES = 3
		};
		enum HQPStatus
		{
			HQP_STATUS_UNKNOWN = -1,
			HQP_STATUS_OPTIMAL = 0,
			HQP_STATUS_INFEASIBLE = 1,
			HQP_STATUS_UNBOUNDED = 2,
			HQP_STATUS_MAX_ITER_REACHED = 3,
			HQP_STATUS_ERROR = 4
		};

		class HQPOutput;
		class SolverHQPBase;

		template<typename T1, typename T2>
		class aligned_pair
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

				aligned_pair(const T1 & t1, const T2 & t2) : first(t1), second(t2) {}

			T1 first;
			T2 second;

		};

		template<typename T1, typename T2>
		inline aligned_pair<T1, T2> make_pair(const T1 & t1, const T2 & t2)
		{
			return aligned_pair<T1, T2>(t1, t2);
		}
		
		typedef aligned_vector< aligned_pair<double, constraint::ConstraintBase*> > ConstraintLevel;
		typedef aligned_vector< aligned_pair<double, const constraint::ConstraintBase*> > ConstConstraintLevel;
		typedef aligned_vector<ConstraintLevel> HQPData;
		typedef aligned_vector<ConstConstraintLevel> ConstHQPData;
	}
}


#endif