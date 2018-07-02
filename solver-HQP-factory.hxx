#ifndef __HQP__Solver_FACTORY__HXX__
#define __HQP__Solver_FACTORY__HXX__

#include "solver-HQP-factory.h"

// not use!!
namespace HQP
{
  namespace solver
  {
    
    template<int nVars, int nEqCon, int nIneqCon>
    SolverHQPBase* SolverHQPFactory::createNewSolver(const SolverHQP solverType, const std::string & name)
    {
      if(solverType==SOLVER_HQP_EIQUADPROG_RT)
        return new SolverHQuadProgRT<nVars, nEqCon, nIneqCon>(name);
      
      assert(false && "Specified solver type not recognized");
      return NULL;
    }
  }
}

#endif // ifndef __HQP__Solver_FACTORY__HXX__
