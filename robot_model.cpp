#include "robot_model.h"
#include <vector>

HQP::robot::RobotModel::RobotModel(Type robottype) {
	model_ = new Model();
	model_->gravity = Eigen::Vector3d(0., 0., -9.81);

	q_rbdl_.resize(dof);
	qdot_rbdl_.resize(dof);
	q_rbdl_.setZero();
	qdot_rbdl_.setZero();

	qddot_rbdl_.resize(dof);
	qddot_rbdl_.setZero();

	m_NLE_.resize(dof);
	m_NLE_.setZero();
	m_Mass_mat_.resize(dof, dof);
	m_Mass_mat_.setZero();
	m_J_.resize(6, dof);
	m_J_.setZero();
	m_pos_.setZero();
	m_Ori_.setZero();
	m_Trans_.linear().setZero();
	m_Trans_.translation().setZero();
	
	tau_.resize(dof);
	tau_.setZero();

	m_robot_type_ = robottype;
	if (m_robot_type_ == 0) {
		m_nq_ = dof;
		m_nv_ = dof;
		m_na_ = dof;
	}
	else if (m_robot_type_ == 1) {
		m_nq_ = dof+4;
		m_nv_ = dof+3;
		m_na_ = dof;
	}
	else if (m_robot_type_ == 2) {
		m_nq_ = dof + 7;
		m_nv_ = dof + 6;
		m_na_ = dof;
	}

	setRobot();
}
HQP::robot::RobotModel::~RobotModel() {

}

void HQP::robot::RobotModel::setRobot() {
	for (int i = 0; i < dof; i++)
		mass_[i] = 1.0;

	axis_[0] = Eigen::Vector3d::UnitZ();
	axis_[1] = Eigen::Vector3d::UnitY();
	axis_[2] = Eigen::Vector3d::UnitZ();
	axis_[3] = -1.0*Eigen::Vector3d::UnitY();
	axis_[4] = Eigen::Vector3d::UnitZ();
	axis_[5] = -1.0*Eigen::Vector3d::UnitY();
	axis_[6] = -1.0*Eigen::Vector3d::UnitZ();

	for (int i = 0; i < dof; i++)
		inertia_[i] = Eigen::Vector3d::Identity() * 0.001;

	joint_position_global_[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	joint_position_global_[1] = joint_position_global_[0];
	joint_position_global_[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	joint_position_global_[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	joint_position_global_[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	joint_position_global_[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	joint_position_global_[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);
	
	joint_position_local_[0] = joint_position_global_[0];
	for (int i=1; i < dof; i++)
		joint_position_local_[i] = joint_position_global_[i] - joint_position_global_[i-1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

	for (int i = 0; i < dof; i++)
		com_position_[i] -=  joint_position_global_[i];

	for (int i = 0; i < dof; i++) {
		body_[i] = Body(mass_[i], com_position_[i], inertia_[i]);
		joint_[i] = Joint(JointTypeRevolute, axis_[i]);

		if (i == 0)
			body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	}
}
void HQP::robot::RobotModel::Jacobian(const int & frame_id) {
	MatrixXd J_temp = m_J_;
	CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], J_temp, true);
	for (int i = 0; i < 2; i++) {
		m_J_.block(i * 3, 0, 3, dof) = J_temp.block(3 - i * 3, 0, 3, dof);
	}
}
void HQP::robot::RobotModel::Position(const int & frame_id) {
	m_pos_ = CalcBodyToBaseCoordinates(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], true);
}
void HQP::robot::RobotModel::Orientation(const int & frame_id) {
	m_Ori_ = CalcBodyWorldOrientation(*model_, q_rbdl_, body_id_[frame_id - 1], true).transpose();
}
void HQP::robot::RobotModel::Transformation(const int & frame_id) {
	Position(frame_id);
	Orientation(frame_id);
	m_Trans_.linear() = m_Ori_;
	m_Trans_.translation() = m_pos_;
}
void HQP::robot::RobotModel::NLEtorque() {
	InverseDynamics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_, tau_);
	for (int i = 0; i < dof; i++)
		m_NLE_(i) = tau_(i);
}
void HQP::robot::RobotModel::MassMatrix() {
	CompositeRigidBodyAlgorithm(*model_, q_rbdl_, m_Mass_mat_, true);
}
void HQP::robot::RobotModel::getUpdateKinematics(const VectorXd & q, const VectorXd & qdot) {
	q_rbdl_ = q;
	qdot_rbdl_ = qdot;
}
void HQP::robot::RobotModel::PointVelocity(const int & frame_id) {
	p_dot_ = CalcPointVelocity6D(*model_, q_rbdl_, qdot_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1]);
	m_p_dot_.angular() = p_dot_.head<3>();
	m_p_dot_.linear() = p_dot_.tail<3>();
}

