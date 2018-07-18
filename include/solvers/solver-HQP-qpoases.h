#ifndef __HQP__Solver_QPOASES__
#define __HQP__Solver_QPOASES__

#include "solvers/solver-HQP-base.h"
#include <qpOASES.hpp>

using namespace qpOASES;
typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> Matrix_Row;
typedef std::size_t IndexSet;

namespace HQP
{
  namespace solver
  {
    class Solver_HQP_qpoases:
        public SolverHQPBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
      Solver_HQP_qpoases(const std::string & name);
      void resize(unsigned int n, unsigned int neq, unsigned int nin, unsigned int nbound);
	    void resize_level(unsigned int level);

	  const HQPOutput & solve(const HQPData & problemData);
      double getObjectiveValue();

    protected:
      void sendMsg(const std::string & s);

      MatrixXd m_H[5], m_A[5];
      VectorXd m_g[5];
      double m_objValue[5];

      Matrix_Row m_H_Row[5], m_A_Row[5];
      VectorXd m_Alb[5], m_Aub[5], m_lb[5], m_ub[5];
	    VectorXd x_sol[5];
			MatrixXd m_CE[5];
			VectorXd m_ce0[5];
			MatrixXd m_CI[5];
			VectorXd m_ci0[5];
      VectorXd m_ci1[5];
	  
	  unsigned int m_h_size;
	  bool m_change[5];

      double m_hessian_regularization;

      Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
	    IndexSet m_activeSetSize;

      // this is for qpoases
      Options m_options;
      SQProblem m_solver[5];
      bool m_init_succeeded;
      returnValue m_status;

#ifdef ELIMINATE_EQUALITY_CONSTRAINTS
      Eigen::CompleteOrthogonalDecomposition<Matrix>  m_CE_dec; 
      Matrix m_ZT_H_Z;
      Matrix m_CI_Z;
#endif

			unsigned int m_neq[5];  /// number of equality constraints
			unsigned int m_neq_t[5]; // number of equality constraints total
			unsigned int m_nin[5];  /// number of inequality constraints
			unsigned int m_nin_t[5];
			unsigned int m_nbound[5];
			unsigned int m_n[5];    /// number of variables
			unsigned int m_slack;
      bool get_dim;
    };
  }
}

#endif // ifndef __HQP__Solver_QPOASES__
