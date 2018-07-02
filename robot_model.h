#ifndef __ROBOT_MODEL__
#define __ROBOT_MODEL__

#include <rbdl/rbdl.h>
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
			const VectorXd & getPosition(const int & frame_id) { 
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
			const MotionVector<double> & getPointVelocity(const int & frame_id) {
				PointVelocity(frame_id);
				return m_p_dot_;
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
			Joint joint_[dof];

			VectorXd q_;
			VectorXd qdot_;
			
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

			MatrixXd m_Mass_mat_;
			VectorXd m_NLE_;
			Vector3d m_pos_;
			Matrix3d m_Ori_;
			MatrixXd m_J_;
			MotionVector<double> m_p_dot_;

			Transform3d m_Trans_;

			Type m_robot_type_;

		protected:
			unsigned int m_nv_;
			unsigned int m_na_;
			unsigned int m_nq_;

		};
	}
}


#endif