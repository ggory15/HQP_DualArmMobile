#ifndef __HQP__TASK__OP_POSTURE__
#define __HQP__TASK__OP_POSTURE__

#include "task-motion.h"
#include "trajectory-base.h"
#include "constraint-equality.h"
#include "motion.h"

namespace HQP
{
  namespace tasks
  {
    class TaskOperationalSpace : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef constraint::ConstraintEquality ConstraintEquality;
	  typedef std::size_t Index;

	  TaskOperationalSpace(const std::string & name, RobotModel & robot, const Index & frameid);

      int dim() const;
      const ConstraintBase & compute(const double t, Cref_vectorXd q, Cref_vectorXd v);

      const ConstraintBase & getConstraint() const;

      void setReference(TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const VectorXd & getDesiredAcceleration() const;
	  VectorXd getAcceleration(Cref_vectorXd dv) const;

      const VectorXd & position_error() const;
      const VectorXd & velocity_error() const;
      const VectorXd & position() const;
      const VectorXd & velocity() const;
      const VectorXd & position_ref() const;
      const VectorXd & velocity_ref() const;

      const VectorXd & Kp() const;
      const VectorXd & Kd() const;
      void Kp(Cref_vectorXd Kp);
      void Kd(Cref_vectorXd Kp);

	  void setSingular(const bool & singular) {
		  m_singular = singular;
	  }
      //Index frame_id() const;

    protected:
      Index m_frame_id;
	  MotionVector<double> m_p_error, m_v_error; // check -> Motion
      VectorXd m_p_error_vec, m_v_error_vec;
      VectorXd m_p, m_v;
	  VectorXd m_p_ref, m_v_ref_vec;
	  MotionVector<double> m_v_ref, m_a_ref;
      Transform3d m_M_ref, m_wMl;
	  VectorXd m_Kp;
	  VectorXd m_Kd;
	  VectorXd m_a_des;
	  MotionVector<double> m_drift;
      Matrix6x m_J;
      ConstraintEquality m_constraint;
      TrajectorySample m_ref;
	  bool m_singular;
    };    
  }
}

#endif // __HQP__TASK__OP_POSTURE__
