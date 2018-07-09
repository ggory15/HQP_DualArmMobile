#include "task-base.h"

namespace HQP
{
  namespace tasks
  {
    TaskBase::TaskBase(const std::string & name, RobotModel & robot):
      m_name(name),
      m_robot(robot)
    {}

    const std::string & TaskBase::name() const
    {
      return m_name;
    }

    void TaskBase::name(const std::string & name)
    {
      m_name = name;
    }    
  }
}
