#ifndef __HQP__TASK__MOTION__
#define __HQP__TASK__MOTION__

#include "tasks/task-base.h"
#include "trajectories/trajectory-base.h"

namespace HQP
{
  namespace tasks
  {
    class TaskMotion : public TaskBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef trajectories::TrajectorySample TrajectorySample;

      TaskMotion(const std::string & name, RobotModel & robot);

      virtual const TrajectorySample & getReference() const = 0;
      virtual const VectorXd & getDesiredAcceleration() const = 0;

      virtual VectorXd getAcceleration(Cref_vectorXd dv) const = 0;

      virtual const VectorXd & position_error() const = 0;
      virtual const VectorXd & velocity_error() const = 0;
      virtual const VectorXd & position() const = 0;
      virtual const VectorXd & velocity() const = 0;
      virtual const VectorXd & position_ref() const = 0;
      virtual const VectorXd & velocity_ref() const = 0;

    };
  }
}

#endif // __HQP__TASK__MOTION__
