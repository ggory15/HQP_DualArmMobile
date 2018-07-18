#include "tasks/task-operational.h"
#include "utils/utils.h"

using namespace std;
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
      m_J.setZero(6, robot.nv());
	  m_singular = false;
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
	
      Transform3d oMi;
      MotionVector<double> v_frame;
      
	  m_robot.getUpdateKinematics(q, v);
	  oMi = m_robot.getTransformation(m_frame_id);

	  v_frame = m_robot.getPointVeloecity(m_frame_id);
	  m_drift.setZero(); // check acc
	  //m_robot.frameClassicAcceleration(data, m_frame_id, m_drift);

      // Transformation from local to world
      m_wMl.linear() = oMi.linear();

	  Transform3d b;
	  b = m_M_ref.inverse() * oMi;
	  m_p_error = log6(b);
	  
	  m_v_error = v_frame - m_v_ref; //check actInv
	  m_v_error = actinv(oMi, m_v_error.vector());
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
	
	  m_J = m_robot.getJacobian(m_frame_id);

	  for (int i = 0; i < m_J.cols(); i++) {
		  m_J.middleCols(i, 1) = actinv(oMi, m_J.middleCols(i, 1)).vector();
	  } // world jacobian to local jacobian

	  MatrixXd J_EE_left(6, 7);
	  J_EE_left = m_J.block(0, 2, 6, 7);

	  Eigen::ColPivHouseholderQR<MatrixXd> qr(J_EE_left);
      MatrixXd U, Z, T;


	  U = qr.matrixQ();
	  VectorXd m_a_des_new;
	  MatrixXd m_J_new;

	  m_J_new = m_J;
	   Z = qr.matrixR().topLeftCorner(6, 7).triangularView<Upper>();
	  T = qr.colsPermutation();

	  if (!m_singular) {
		//  if (qr.matrixR().topLeftCorner(6, 7).triangularView<Upper>()(5, 5) < 0.05) {
			  m_constraint.resize(5, m_robot.nv());
			  m_a_des_new = U.topLeftCorner(6, 5).transpose() * m_a_des;			 
			  m_J_new = U.topLeftCorner(6, 5).transpose() * m_J;

			  m_constraint.setMatrix(m_J_new);
			  m_constraint.setVector(m_a_des_new);
			  return m_constraint;
		 // }
		  //else {
			 // m_constraint.setMatrix(m_J);
			 // m_constraint.setVector(m_a_des);
			 // return m_constraint;
		  //}
	  }
	  else {
		  //if (qr.matrixR().topLeftCorner(6, 7).triangularView<Upper>()(5, 5) < 0.05) {
			  m_J_new.resize(1, 16);
			  m_J_new.setZero();
			  m_constraint.resize(1, m_robot.nv());

			  VectorXd new_a_set(1);
			  MatrixXd new_J_set(1, 16);
			  new_J_set = U.topRightCorner(6, 1).transpose() * m_J;
			  double h_input = fabs(qr.matrixR().topLeftCorner(6, 7).triangularView<Upper>()(5, 5));
			  m_a_des_new = h_factor(h_input, 0.1, 0.05) * U.topRightCorner(6, 1).transpose() * m_a_des;
			 // cout << "h_facetor" << h_input << endl;

			 /* for (int i = 0; i < 2; i++)
				  m_J_new(0, i) = new_J_set(0, i);*/

			  //for (int i = 0; i < 14; i++)
			//	  m_J_new(0, i + 2) = new_J_set(0, i + 2);
			  //m_J_new.topRows(1).head(2) = new_J_set.topRows(1).head(2).transpose();
			//  m_J_new.bottomRows(1).tail(14) = new_J_set.tail(14);

			  //new_a_set(0) = 0.0;// (U.topRightCorner(6, 1).transpose() * m_a_des)(0, 0);
			  new_a_set(0) = 0.0;// h_factor(qr.matrixR().topLeftCorner(6, 7).triangularView<Upper>()(5, 5), 0.05, 0.005) * (U.topRightCorner(6, 1).transpose() * m_a_des)(0, 0);

			//  m_a_des_new = U.topRightCorner(6, 1).transpose() * m_a_des;
			 // m_J_new = U.topRightCorner(6, 1).transpose() * m_J;


			  m_constraint.setMatrix(new_J_set);
			  m_constraint.setVector(m_a_des_new);
			  return m_constraint;
		  }
		  //else {
			 // m_constraint.setMatrix(m_J);
			 // m_constraint.setVector(m_a_des);
			 // return m_constraint;
		  //}
	 //}	
	  //m_constraint.setMatrix(m_J);
	  //m_constraint.setVector(m_a_des);
	  //return m_constraint;
    }    
  }
}
