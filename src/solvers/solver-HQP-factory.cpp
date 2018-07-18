#include "solvers/solver-HQP-factory.h"
//#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
//#include <tsid/solvers/solver-HQP-eiquadprog-fast.hpp>
#include "solvers/solver-HQP-qpoases.h"
#include "solvers/solver-HQP-eiquadprog.h"

namespace HQP
{
  namespace solver
  {
    
    SolverHQPBase* SolverHQPFactory::createNewSolver(const SolverHQP solverType,
                                                     const std::string & name)
    {
      if(solverType==SOLVER_HQP_EIQUADPROG)
        return new SolverHQuadProg(name);
      
      //if(solverType==SOLVER_HQP_EIQUADPROG_FAST)
      //  return new SolverHQuadProgFast(name);
      //
      if(solverType==SOLVER_HQP_QPOASES)
        return new Solver_HQP_qpoases(name);
      
      assert(false && "Specified solver type not recognized");
      return NULL;
    }    
  }
}
