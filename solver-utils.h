#ifndef __HQP__Solver_UTILS__
#define __HQP__Solver_UTILS__

#include "fwd_solver.h"
#include "inverse-dynamics.h"

#include <string>

namespace HQP
{
  namespace solver
  {
    std::string HQPDataToString(const HQPData & data, bool printMatrices=false);
    std::string InvDynToString2(InverseDynamics & inv_dyn, bool printMatrices=false);
  }  
}

#endif // ifndef __HQP__Solver_UTILS__
