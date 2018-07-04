#include "task-operational.h"
#include "utils.h"

namespace HQP
{
  namespace tasks
  {
    using namespace constraint;
    using namespace trajectories;

	TaskOperationalSpace::TaskOperationalSpace(const std::string & name, RobotModel & robot, const Index & frameid):
      TaskMotion(name, robot),
      m_frame_id(frameid),
      m_constraint(name, 6, robot.nv()),
      m_ref(12, 6)
    {
      m_v_ref.setZero();
      m_a_ref.setZero();
      m_M_ref.setIdentity();
      m_wMl.setIdentity();
      m_p_error_vec.setZero(6);
      m_v_error_vec.setZero(6);
      m_p.resize(12);
      m_v.resize(6);
      m_p_ref.resize(12);
      m_v_ref_vec.resize(6);
      m_Kp.setZero(6);
      m_Kd.setZero(6);
      m_a_des.setZero(6);
	  m_v_des.setZero(dof);
      m_J.setZero(6, robot.nv());
    }

    int TaskOperationalSpace::dim() const
    {
      return 6;
    }

    const VectorXd & TaskOperationalSpace::Kp() const { return m_Kp; }

    const VectorXd & TaskOperationalSpace::Kd() const { return m_Kd; }

    void TaskOperationalSpace::Kp(Cref_vectorXd Kp)
    {
      assert(Kp.size()==6);
      m_Kp = Kp;
    }

    void TaskOperationalSpace::Kd(Cref_vectorXd Kd)
    {
      assert(Kd.size()==6);
      m_Kd = Kd;
    }
		
    void TaskOperationalSpace::setReference(TrajectorySample & ref)
    {
      m_ref = ref;
	  m_M_ref.translation() = ref.pos.head<3>();
	  typedef Eigen::Matrix<double, 3, 3> Matrix3;
	  m_M_ref.linear()= Eigen::Map<const Matrix3>(&ref.pos(3), 3, 3);

      m_v_ref = MotionVector<double>(ref.vel);
      m_a_ref = MotionVector<double>(ref.acc);
    }

    const TrajectorySample & TaskOperationalSpace::getReference() const
    {
      return m_ref;
    }

    const VectorXd & TaskOperationalSpace::position_error() const
    {
      return m_p_error_vec;
    }

    const VectorXd & TaskOperationalSpace::velocity_error() const
    {
      return m_v_error_vec;
    }

    const VectorXd & TaskOperationalSpace::position() const
    {
      return m_p;
    }

    const VectorXd & TaskOperationalSpace::velocity() const
    {
      return m_v;
    }

    const VectorXd & TaskOperationalSpace::position_ref() const
    {
      return m_p_ref;
    }

    const VectorXd & TaskOperationalSpace::velocity_ref() const
    {
      return m_v_ref_vec;
    }

    const VectorXd & TaskOperationalSpace::getDesiredAcceleration() const
    {
      return m_a_des;
    }

	VectorXd TaskOperationalSpace::getAcceleration(Cref_vectorXd dv) const
    {
      return m_constraint.matrix()*dv + m_drift.vector();
    }

    //Index TaskOperationalSpace::frame_id() const
    //{
    //  return m_frame_id;
    //}

    const ConstraintBase & TaskOperationalSpace::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskOperationalSpace::compute(const double t, Cref_vectorXd q, Cref_vectorXd v)
    {

	#ifdef JOINTCTRL

		Transform3d oMi;
		MotionVector<double> v_frame;

		m_robot.getUpdateKinematics(q, v);
		oMi = m_robot.getTransformation(m_frame_id);
		v_frame = m_robot.getPointVelocity(m_frame_id);
		m_drift.setZero(); // check acc
						   //m_robot.frameClassicAcceleration(data, m_frame_id, m_drift);

						   // Transformation from local to world
		m_wMl.linear() = oMi.linear();

		Transform3d b;
		b = m_M_ref.inverse() * oMi;

		m_p_error = log6(b);
		m_v_error = v_frame - m_v_ref.actInv(oMi); //check actInv

		m_p_error_vec = m_p_error.vector();
		m_v_error_vec = m_v_error.vector();

		m_p_ref.head<3>() = m_M_ref.translation();
		typedef Eigen::Matrix<double, 9, 1> Vector9;
		m_p_ref.tail<9>() = Eigen::Map<const Vector9>(&m_M_ref.rotation()(0), 9);

		m_v_ref_vec = m_v_ref.vector();

		m_p.head<3>() = oMi.translation();
		m_p.tail<9>() = Eigen::Map<const Vector9>(&oMi.rotation()(0), 9);

		m_v = v_frame.vector();

		m_v_des = -m_Kp.cwiseProduct(m_p_error.vector())
			- m_Kd.cwiseProduct(m_v_error.vector());

		  m_J = m_robot.getJacobian(m_frame_id); //check world jacobian
		  m_constraint.setMatrix(m_J);
		  m_constraint.setVector(m_v_des);


	#else

	  Transform3d oMi;
	  MotionVector<double> v_frame;

	  m_robot.getUpdateKinematics(q, v);
	  oMi = m_robot.getTransformation(m_frame_id);
	  v_frame = m_robot.getPointVelocity(m_frame_id);
	  m_drift.setZero(); // check acc
						 //m_robot.frameClassicAcceleration(data, m_frame_id, m_drift);

						 // Transformation from local to world
	  m_wMl.linear() = oMi.linear();

	  Transform3d b;
	  b = m_M_ref.inverse() * oMi;

	  m_p_error = log6(b);
	  m_v_error = v_frame - m_v_ref.actInv(oMi); //check actInv

	  m_p_error_vec = m_p_error.vector();
	  m_v_error_vec = m_v_error.vector();

	  m_p_ref.head<3>() = m_M_ref.translation();
	  typedef Eigen::Matrix<double, 9, 1> Vector9;
	  m_p_ref.tail<9>() = Eigen::Map<const Vector9>(&m_M_ref.rotation()(0), 9);

	  m_v_ref_vec = m_v_ref.vector();

	  m_p.head<3>() = oMi.translation();
	  m_p.tail<9>() = Eigen::Map<const Vector9>(&oMi.rotation()(0), 9);

	  m_v = v_frame.vector();

	  m_a_des = -m_Kp.cwiseProduct(m_p_error.vector())
		  - m_Kd.cwiseProduct(m_v_error.vector())
		  + m_a_ref.actInv(m_wMl).vector();

	  m_J = m_robot.getJacobian(m_frame_id); //check world jacobian

	  m_constraint.setMatrix(m_J);
	  m_constraint.setVector(m_a_des);


	#endif // JOINTCTRL
      return m_constraint;
    }    
  }
}
