#ifndef __HQP__TASK__COM__
#define __HQP__TASK__COM__

/*
#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"

namespace tsid
{
  namespace tasks
  {

    class TaskComEquality : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Vector3 Vector3;
      typedef math::ConstraintEquality ConstraintEquality;

      TaskComEquality(const std::string & name,
                      RobotWrapper & robot);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     const Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const Vector & getDesiredAcceleration() const;
      Vector getAcceleration(ConstRefVector dv) const;

      const Vector & position_error() const;
      const Vector & velocity_error() const;
      const Vector & position() const;
      const Vector & velocity() const;
      const Vector & position_ref() const;
      const Vector & velocity_ref() const;

      const Vector3 & Kp();
      const Vector3 & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      Vector3 m_Kp;
      Vector3 m_Kd;
      Vector3 m_p_error, m_v_error;
      Vector3 m_a_des;
      Vector m_a_des_vec;
      Vector3 m_drift;
      Vector m_p_com, m_v_com;
      Vector m_p_error_vec, m_v_error_vec;
      TrajectorySample m_ref;
      ConstraintEquality m_constraint;
    };
  }
}
*/

#endif // ifndef __invdyn_task_com_equality_hpp__
