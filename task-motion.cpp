#include "task-motion.h"

namespace HQP
{
  namespace tasks
  {
    TaskMotion::TaskMotion(const std::string & name, RobotModel & robot):
      TaskBase(name, robot)
    {}
  }
}
