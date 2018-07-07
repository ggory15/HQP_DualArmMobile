#ifndef __ROBOT_MODEL__
#define __ROBOT_MODEL__

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_config.h>
//#include <rbdl/rbdl.h>
#include "motion.h"
#include "fwd.h"

using namespace RigidBodyDynamics;
using namespace Eigen;


namespace HQP {
	namespace robot {

		enum Type {
			Manipulator = 0,
			MobileManipulator = 1,
			Humanoid = 2
		};

		class RobotModel
		{			
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			RobotModel(Type robottype);
			~RobotModel();

			void getUpdateKinematics(const VectorXd & q, const VectorXd & qdot);
			const MatrixXd & getJacobian(const int & frame_id) { 
				Jacobian(frame_id);
				return m_J_; 
			}
			const Vector3d & getPosition(const int & frame_id) { 
				Position(frame_id);
				return m_pos_;
			}
			const MatrixXd & getOrientation(const int & frame_id) { 
				Orientation(frame_id);
				return m_Ori_;
			}
			const Transform3d & getTransformation(const int & frame_id) { 
				Transformation(frame_id);
				return m_Trans_; 
			}
			const VectorXd & getNLEtorque() { 
				NLEtorque();
				return m_NLE_; 
			}
			const MatrixXd & getMassMatrix() { 
				MassMatrix();
				return m_Mass_mat_; 
			}
			const unsigned int & na() {
				return m_na_;
			}
			const unsigned int & nv() {
				return m_nv_;
			}
			const unsigned int & nq() {
				return m_nq_;
			}
			const Type & type() {
				return m_robot_type_;
			}
			const MotionVector<double> & getPointVeloecity(const int & frame_id) {
				PointVelocity(frame_id);
				return m_p_dot_;
			}
			const VectorXd & getRealJointPosition() {
				q_real_.tail(m_nv_) = q_rbdl_.tail(m_nv_);
				return q_real_;
			}
			const VectorXd & getRealJointVelocity() {
				qdot_real_.tail(m_nv_) = qdot_rbdl_.tail(m_nv_);
				return qdot_real_;
			}
			const Transform3d & getMobileTransformation() {
				return m_base_;
			}
			const MotionVector<double> & getMobileVelocity() {
				return m_mobile_dot_;
			}

		private:			
			void Jacobian(const int & frame_id);
			void Position(const int & frame_id);
			void Orientation(const int & frame_id);
			void Transformation(const int & frame_id);
			void NLEtorque();
			void MassMatrix();
			void PointVelocity(const int & frame_id);
			void setRobot();

			Model* model_;
			Body body_[dof];
			Body virtual_body_[6];
			Body base_;

			Joint joint_[dof];
			Joint virtual_joint_[6];
		
			VectorXd q_;
			VectorXd qdot_;

			VectorXd q_real_;
			VectorXd qdot_real_;
			
			double mass_[dof];
			Math::Vector3d axis_[dof];
			Math::Vector3d inertia_[dof];
			Math::Vector3d joint_position_global_[dof];
			Math::Vector3d joint_position_local_[dof];
			Math::Vector3d com_position_[dof];
			Math::SpatialVector p_dot_;

			Math::VectorNd q_rbdl_;
			Math::VectorNd qdot_rbdl_;
			Math::VectorNd qddot_rbdl_;
			Math::VectorNd tau_;

			unsigned int body_id_[dof];		
			unsigned int base_id_; // virtual joint
			unsigned int virtual_body_id_[6];

			MatrixXd m_Mass_mat_;
			VectorXd m_NLE_;
			Vector3d m_pos_;
			Matrix3d m_Ori_;
			MatrixXd m_J_;
			MotionVector<double> m_p_dot_;
			MotionVector<double> m_mobile_dot_;
			MatrixXd m_selection_;
			MatrixXd m_selection_dot_;

			Transform3d m_Trans_;
			Transform3d m_base_;

			Type m_robot_type_;

		protected:
			unsigned int m_nv_;
			unsigned int m_na_;
			unsigned int m_nq_;

		};
	}
}


#endif