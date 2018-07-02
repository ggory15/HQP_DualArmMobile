#ifndef __HQP__Solver_QPOASES__
#define __HQP__Solver_QPOASES__

#include "solver-HQP-base.h"
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
      void resize(unsigned int n, unsigned int neq, unsigned int nin);

	  const HQPOutput & solve(const HQPData & problemData);
      double getObjectiveValue();

    protected:
      void sendMsg(const std::string & s);

      MatrixXd m_H;
      VectorXd m_g;
      MatrixXd m_CE;
      VectorXd m_ce0;
      MatrixXd m_CI;
      VectorXd m_ci0;
      double m_objValue;

      Matrix_Row m_H_Row, m_A_Row;
      VectorXd m_Alb, m_Aub;

      double m_hessian_regularization;

      Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
	  IndexSet m_activeSetSize;

      // this is for qpoases
      Options m_options;
      SQProblem m_solver;
      bool m_init_succeeded;
      returnValue m_status;

#ifdef ELIMINATE_EQUALITY_CONSTRAINTS
      Eigen::CompleteOrthogonalDecomposition<Matrix>  m_CE_dec; 
      Matrix m_ZT_H_Z;
      Matrix m_CI_Z;
#endif

      unsigned int m_neq;  /// number of equality constraints
      unsigned int m_nin;  /// number of inequality constraints
      unsigned int m_n;    /// number of variables
    };
  }
}

#endif // ifndef __HQP__Solver_QPOASES__
