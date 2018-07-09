#ifndef __HQP__TASK__BASE__
#define __HQP__TASK__BASE__

#include "fwd_constraints.h"
#include "robot_model.h"
#include "constraint-base.h"
#include "fwd.h"

namespace HQP
{
  namespace tasks
  {
    class TaskBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef constraint::ConstraintBase ConstraintBase;
      typedef robot::RobotModel RobotModel;

      TaskBase(const std::string & name, RobotModel & robot);

      const std::string & name() const;
      void name(const std::string & name);
      
      virtual int dim() const = 0;
      virtual const ConstraintBase & compute(const double t, Cref_vectorXd q, Cref_vectorXd v) = 0;
      virtual const ConstraintBase & getConstraint() const = 0;
      
    protected:
      std::string m_name;
	  RobotModel & m_robot;
    };
  }
}

#endif // __HQP__TASK__BASE__
