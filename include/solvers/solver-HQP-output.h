#ifndef __HQP__Solver_OUTPUT__
#define __HQP__Solver_OUTPUT__

#include "solvers/fwd_solver.h"
#include "constraint/fwd_constraints.h"

#include <vector>
namespace HQP
{
  namespace solver
  {
    class HQPOutput
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      HQPStatus status;    /// solver status
      VectorXd x;            /// solution
      VectorXd lambda;       /// Lagrange multipliers
      VectorXi activeSet;  /// indexes of active inequalities
      int iterations;      /// number of iterations performed by the solver
      
      HQPOutput(){}
      
      HQPOutput(int nVars, int nEqCon, int nInCon)
      {
        resize(nVars, nEqCon, nInCon);
      }
      
      void resize(int nVars, int nEqCon, int nInCon)
      {
        x.resize(nVars);
        lambda.resize(nEqCon+nInCon);
        activeSet.resize(nInCon);
      }
    };
  }
}

#endif // ifndef __HQP__Solver_OUTPUT__
