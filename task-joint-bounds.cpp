#include "task-joint-bounds.h"
namespace HQP
{
  namespace tasks
  {
	  using namespace constraint;
	  using namespace trajectories;

	  TaskJointLimit::TaskJointLimit(const std::string & name, RobotModel & robot) 
		  : TaskBase(name, robot), m_constraint(name) {
		  m_robot = robot;
		  if (robot.type() == 0) {
			  m_constraint.setLowerBound(-200.0 * VectorXd(robot.nv()).setOnes());
			  m_constraint.setUpperBound(200.0 * VectorXd(robot.nv()).setOnes());
			  m_Kp.setZero(robot.nv());
			  m_Kd.setZero(robot.nv());
			  VectorXd m = VectorXd::Ones(robot.nv());
			  mask(m);
		  }

		  m_buffer = 5.0 * M_PI / 180.0;
	  }

	  int TaskJointLimit::dim() const
	  {
		  return m_robot.nv();
	  }

	  const ConstraintBase & TaskJointLimit::compute(const double t, Cref_vectorXd q, Cref_vectorXd v) {
		  if (m_robot.type() == 0) {
			  for (int i = 0; i < m_robot.nv(); i++) {
				  if (q(i) < m_q_lbound(i) + m_buffer) {
					  m_constraint.lowerBound()(i) = m_Kp(i) * ((m_q_lbound(i) + m_buffer) - q(i)) - m_Kd(i) * v(i);
					  m_constraint.upperBound()(i) = 200.0;
				  }
				  else if (q(i) > m_q_ubound(i) - m_buffer) {
					  m_constraint.upperBound()(i) = m_Kp(i) * ((m_q_ubound(i) - m_buffer) - q(i)) - m_Kd(i) * v(i);
					  m_constraint.lowerBound()(i) = -200.0;
				  }
				  else {
					  m_constraint.upperBound()(i) = 200.0;
					  m_constraint.lowerBound()(i) = -200.0;
				  }
			  }

			  return m_constraint;
		  }

	  }
	  const ConstraintBase & TaskJointLimit::getConstraint() const{
		  return m_constraint;
	  }

	  void TaskJointLimit::setJointLimit(Cref_vectorXd q_low, Cref_vectorXd q_high) {
		  if (m_robot.type() == 0)
			assert(q_low.size() == m_robot.nv() && q_high.size() == m_robot.nv());

		  m_q_lbound = q_low;
		  m_q_ubound = q_high;
	  }

	  const  VectorXd & TaskJointLimit::mask() const {
		
	  }
	  void TaskJointLimit::mask(const VectorXd & mask) {
		 
	  }

	  const VectorXd & TaskJointLimit::Kp() { return m_Kp; }

	  const VectorXd & TaskJointLimit::Kd() { return m_Kd; }

	  void TaskJointLimit::Kp(Cref_vectorXd Kp)
	  {
		  if (m_robot.type() == 0)
			  assert(Kp.size() == m_robot.nv());

		  m_Kp = Kp;
	  }

	  void TaskJointLimit::Kd(Cref_vectorXd Kd)
	  {
		  if (m_robot.type() == 0)
			  assert(Kd.size() == m_robot.nv());

		  m_Kd = Kd;
	  }    
  }
}
