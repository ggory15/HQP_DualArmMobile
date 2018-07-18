#ifndef __HQP__Solver_BASE__
#define __HQP__Solver_BASE__

#include "solvers/fwd_solver.h"
#include "solvers/solver-HQP-output.h"
#include "constraint/constraint-base.h"

#include <vector>
#include <utility>

namespace HQP
{
  namespace solver
  {

    class SolverHQPBase
    {
    public:
      
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      static std::string const HQP_status_string [5];

      SolverHQPBase(const std::string & name);

      virtual const std::string & name() { return m_name; }

      virtual void resize(unsigned int n, unsigned int neq, unsigned int nin, unsigned int nbound) = 0;

      virtual const HQPOutput & solve(const HQPData & problemData) = 0;

      /** Get the objective value of the last solved problem. */
      virtual double getObjectiveValue() = 0;

      /** Return true if the solver is allowed to warm start, false otherwise. */
      virtual bool getUseWarmStart(){ return m_useWarmStart; }
      /** Specify whether the solver is allowed to use warm-start techniques. */
      virtual void setUseWarmStart(bool useWarmStart){ m_useWarmStart = useWarmStart; }

      /** Get the current maximum number of iterations performed by the solver. */
      virtual unsigned int getMaximumIterations(){ return m_maxIter; }
      /** Set the current maximum number of iterations performed by the solver. */
      virtual bool setMaximumIterations(unsigned int maxIter);


      /** Get the maximum time allowed to solve a problem. */
      virtual double getMaximumTime(){ return m_maxTime; }
      /** Set the maximum time allowed to solve a problem. */
      virtual bool setMaximumTime(double seconds);

    protected:
      
      std::string           m_name;
      bool                  m_useWarmStart;   // true if the solver is allowed to warm start
      int                   m_maxIter;        // max number of iterations
      double                m_maxTime;        // max time to solve the HQP [s]
      HQPOutput             m_output;
    };

  }
}

#endif // ifndef __HQP__Solver_BASE__
