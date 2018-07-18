#include "tasks/task-joint-posture.h"

namespace HQP
{
  namespace tasks
  {
    using namespace constraint;
    using namespace trajectories;

	TaskJointPosture::TaskJointPosture(const std::string & name, RobotModel & robot)
		: TaskMotion(name, robot), m_constraint(name), m_ref(robot.nv())
    {
		if (robot.type() == 0) {
			m_constraint.setMatrix(Eigen::MatrixXd(robot.nv(), robot.nv()).setZero());
			m_Kp.setZero(robot.nv());
			m_Kd.setZero(robot.nv());
			VectorXd m = VectorXd::Ones(robot.nv());
			mask(m);
		}
		else if (robot.type() == 1) {
			m_ref.resize(robot.nv()-2);
			m_constraint.setMatrix(Eigen::MatrixXd(robot.nv()-2, robot.nv()).setZero());
			m_Kp.setZero(robot.nv()-2);
			m_Kd.setZero(robot.nv()-2);
			VectorXd m = VectorXd::Ones(robot.nv()-2);
			mask(m);
		}
		else if (robot.type() == 2) {
			m_ref.resize(robot.nv()-6);
			m_constraint.setMatrix(Eigen::MatrixXd(robot.nv()-6, robot.nv()).setZero());
			m_Kp.setZero(robot.nv()-6);
			m_Kd.setZero(robot.nv()-6);
			VectorXd m = VectorXd::Ones(robot.nv()-6);
			mask(m);
		}
    }

    const VectorXd & TaskJointPosture::mask() const
    {
      return m_mask;
    }

    void TaskJointPosture::mask(const VectorXd & m)
    {
		if (m_robot.type() == 0) {
			assert(m.size() == m_robot.nv());
			m_mask = m;
			const VectorXi::Index dim = static_cast<VectorXi::Index>(m.sum());
			MatrixXd S = MatrixXd::Zero(dim, m_robot.nv());
			m_activeAxes.resize(dim);
			unsigned int j = 0;
			for (unsigned int i = 0; i < m.size(); i++)
				if (m(i) != 0.0)
				{
					assert(m(i) == 1.0);
					S(j, i) = 1.0;
					m_activeAxes(j) = i;
					j++;
				}
			m_constraint.resize((unsigned int)dim, m_robot.nv());
			m_constraint.setMatrix(S);
		}
		if (m_robot.type() == 1) {
			assert(m.size() == m_robot.nv() - 2);
			m_mask = m;
			const VectorXi::Index dim = static_cast<VectorXi::Index>(m.sum());
			MatrixXd S = MatrixXd::Zero(dim, m_robot.nv());
			m_activeAxes.resize(dim);
			unsigned int j = 0;
			for (unsigned int i = 0; i < m.size(); i++)
				if (m(i) != 0.0)
				{
					assert(m(i) == 1.0);
					S(j, 2 + i) = 1.0;
					m_activeAxes(j) = i;
					j++;
				}
			m_constraint.resize((unsigned int)dim, m_robot.nv());
			m_constraint.setMatrix(S);
		}
    }

    int TaskJointPosture::dim() const
    {
      return (int)m_mask.sum();
    }

    const VectorXd & TaskJointPosture::Kp(){ return m_Kp; }

    const VectorXd & TaskJointPosture::Kd(){ return m_Kd; }

    void TaskJointPosture::Kp(Cref_vectorXd Kp)
    {
      //assert(Kp.size()==m_robot.nv()-6);
      m_Kp = Kp;
    }

    void TaskJointPosture::Kd(Cref_vectorXd Kd)
    {
      //assert(Kd.size()==m_robot.nv()-6);
      m_Kd = Kd;
    }

    void TaskJointPosture::setReference(const TrajectorySample & ref)
    {
      //assert(ref.pos.size()==m_robot.nv()-6);
      //assert(ref.vel.size()==m_robot.nv()-6);
      //assert(ref.acc.size()==m_robot.nv()-6);
      m_ref = ref;
    }

    const TrajectorySample & TaskJointPosture::getReference() const
    {
      return m_ref;
    }

    const VectorXd & TaskJointPosture::getDesiredAcceleration() const
    {
      return m_a_des;
    }

	VectorXd TaskJointPosture::getAcceleration(Cref_vectorXd dv) const
    {
      return m_constraint.matrix()*dv;
    }

    const VectorXd & TaskJointPosture::position_error() const
    {
      return m_p_error;
    }

    const VectorXd & TaskJointPosture::velocity_error() const
    {
      return m_v_error;
    }

    const VectorXd & TaskJointPosture::position() const
    {
      return m_p;
    }

    const VectorXd & TaskJointPosture::velocity() const
    {
      return m_v;
    }

    const VectorXd & TaskJointPosture::position_ref() const
    {
      return m_ref.pos;
    }

    const VectorXd & TaskJointPosture::velocity_ref() const
    {
      return m_ref.vel;
    }

    const ConstraintBase & TaskJointPosture::getConstraint() const
    {
      return m_constraint;
    }

	const ConstraintBase & TaskJointPosture::compute(const double, Cref_vectorXd q, Cref_vectorXd v)
	{
		if (m_robot.type() == 0) {
			m_p = q.tail(m_robot.nv());
			m_v = v.tail(m_robot.nv());

			m_p_error = m_p - m_ref.pos;
			m_v_error = m_v - m_ref.vel;

			m_a_des = -m_Kp.cwiseProduct(m_p_error)
				- m_Kd.cwiseProduct(m_v_error)
				+ m_ref.acc;

			for (unsigned int i = 0; i < m_activeAxes.size(); i++)
				m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
			return m_constraint;
		}
		else if (m_robot.type() == 1) {
			m_p = q.tail(m_robot.nv() - 2);
			m_v = v.tail(m_robot.nv() - 2);

			m_p_error = m_p - m_ref.pos;
			m_v_error = m_v - m_ref.vel;

			m_a_des = -m_Kp.cwiseProduct(m_p_error)
				- m_Kd.cwiseProduct(m_v_error)
				+ m_ref.acc;

			for (unsigned int i = 0; i < m_activeAxes.size(); i++)
				m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
			return m_constraint;
		}
	}
  }
}
