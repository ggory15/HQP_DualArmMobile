#ifndef __SOLVER_HQP_EIQUADPROG__
#define __SOLVER_HQP_EIQUADPROG__

#include "solver-HQP-base.h"
typedef std::size_t IndexSet;

namespace HQP
{
	namespace solver
	{
		class  SolverHQuadProg : public SolverHQPBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			SolverHQuadProg(const std::string & name);

			void resize(unsigned int n, unsigned int neq, unsigned int nin);
			void resize(unsigned int n, unsigned int neq, unsigned int nin, unsigned int nbound);
			void resize_level(unsigned int level);
			/** Solve the given Hierarchical Quadratic Program
			*/
			const HQPOutput & solve(const HQPData & problemData);

			/** Get the objective value of the last solved problem. */
			double getObjectiveValue();
			double getObjectiveValue2(unsigned int level);

		protected:

			void sendMsg(const std::string & s);

			MatrixXd m_H[5];
			VectorXd m_g[5];
			MatrixXd m_CE[5];
			VectorXd m_ce0[5];
			MatrixXd m_CI[5];
			VectorXd m_ci0[5];
			double m_objValue[5];
			bool m_change[5];
			VectorXd x_sol[5];

			double m_hessian_regularization;

			Eigen::VectorXi m_activeSet[5];  /// vector containing the indexes of the active inequalities
			Eigen::Index m_activeSetSize[5];

#ifdef ELIMINATE_EQUALITY_CONSTRAINTS
			//      Eigen::FullPivLU<Matrix>                        m_CE_dec;
			//	  Eigen::ColPivHouseholderQR<Matrix>              m_CE_dec; // fast, but difficult to retrieve null space basis
			//      Eigen::FullPivHouseholderQR<Matrix>             m_CE_dec; // doc says it is slow 
			Eigen::CompleteOrthogonalDecomposition<Matrix>  m_CE_dec; // available from Eigen 3.3.0, 40 us for decomposition, 40 us to get null space basis, 40 us to project Hessian
																	  //      Eigen::JacobiSVD<Matrix, Eigen::HouseholderQRPreconditioner> m_CE_dec; // too slow
			Matrix m_ZT_H_Z;
			Matrix m_CI_Z;
#endif

			unsigned int m_neq[5];  /// number of equality constraints
			unsigned int m_nin[5];  /// number of inequality constraints
			unsigned int m_nbound[5];
			unsigned int m_n[5];    /// number of variables
			unsigned int m_slack;
			
		};
	}
}

#endif // ifndef __invdyn_solvers_hqp_eiquadprog_hpp__
