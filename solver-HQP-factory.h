#ifndef __HQP__Solver_FACTORY__
#define __HQP__Solver_FACTORY__

#include "solver-HQP-base.h"

namespace HQP
{
  namespace solver
  {
    
    struct SolverHQPFactory
    {
      
      static SolverHQPBase * createNewSolver(const SolverHQP solverType, const std::string & name);
      template<int nVars, int nEqCon, int nIneqCon>
      static SolverHQPBase* createNewSolver(const SolverHQP solverType,  const std::string & name);
    };
    
  }
}

#endif // ifndef __HQP__Solver_FACTORY__
