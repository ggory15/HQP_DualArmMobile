#include "robot/robot_model.h"
#include <vector>

using namespace std;

HQP::robot::RobotModel::RobotModel(int robottype) {
	model_ = new Model();
	model_->gravity = Eigen::Vector3d(0., 0., -9.81);

	m_robot_type_ = robottype;
	if (m_robot_type_ == 0) {
		m_nq_ = dof;
		m_nv_ = dof;
		m_na_ = dof;
	}
	else if (m_robot_type_ == 1) {
		m_nq_ = dof + 2;
		m_nv_ = dof + 2;
		m_na_ = dof;
	}
	else if (m_robot_type_ == 2) {
		m_nq_ = dof + 6;
		m_nv_ = dof + 6;
		m_na_ = dof;
	}

	q_rbdl_.resize(m_na_ + 5);
	qdot_rbdl_.resize(m_na_ + 5);
	q_rbdl_.setZero();
	qdot_rbdl_.setZero();

	qddot_rbdl_.resize(m_na_ + 5);
	qddot_rbdl_.setZero();

	m_NLE_.resize(m_na_ + 2); // 
	m_NLE_.setZero();
	m_Mass_mat_.resize(m_na_ + 2, m_na_ + 2); // 
	m_Mass_mat_.setZero();
	m_J_.resize(6, m_na_ + 2); // 
	m_J_.setZero();

	m_Ori_.resize(3, 3);
	m_pos_.setZero();
	m_Ori_.setZero();
	m_Trans_.linear().setZero();
	m_Trans_.translation().setZero();

	tau_.resize(m_na_ + 5);
	tau_.setZero();

	q_real_.resize(m_nv_);
	q_real_.setZero();
	qdot_real_.resize(m_nv_);
	qdot_real_.setZero();

	m_selection_.resize(5 + m_na_, m_na_ + 2);
	m_selection_.setZero();
	m_selection_dot_.resize(5 + m_na_, m_na_ + 2);
	m_selection_dot_.setZero();
	m_Mass_virtual_mat_.resize(5 + m_na_, 5 + m_na_);
	m_Mass_virtual_mat_.setZero();
	setRobot();
}
HQP::robot::RobotModel::~RobotModel() {

}

void HQP::robot::RobotModel::setRobot() {
	// for floating base //
	virtual_body_[0] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	virtual_joint_[0] = Joint(JointTypePrismatic, Eigen::Vector3d::UnitX());
	virtual_body_[1] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	virtual_joint_[1] = Joint(JointTypePrismatic, Eigen::Vector3d::UnitY());
	virtual_body_[2] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	virtual_joint_[2] = Joint(Math::SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));

	virtual_body_id_[0] = model_->AddBody(0, Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[0], virtual_body_[0]);
	virtual_body_id_[1] = model_->AddBody(virtual_body_id_[0], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[1], virtual_body_[1]);


	double mass = 30.0;
	base_ = Body(mass, Math::Vector3d(0.0, 0.0, 0.0), Math::Vector3d(0.5, 100.0, 100.0));
	base_id_ = model_->AddBody(virtual_body_id_[1], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[2], base_); //body frame = joint frame
	
	//// for wheel
	virtual_body_[3] = Body(2.6 *2.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.001, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1));
	virtual_joint_[3] = Joint(JointTypeRevolute, -Eigen::Vector3d::UnitY());
	virtual_body_[4] = Body(2.6*2.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.001, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1));
	virtual_joint_[4] = Joint(JointTypeRevolute, 1.0*Eigen::Vector3d::UnitY());

	model_->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, 0.25, 0.165)), virtual_joint_[3], virtual_body_[3]);
	model_->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, -0.25, 0.165)), virtual_joint_[4], virtual_body_[4]);
	////////////////////////////////////////////////////////////////////////////////////////////////////
	
	for (int i = 0; i < dof; i++)
		mass_[i] = 1.0;

	for (int i = 0; i < dof; i++)
		inertia_[i] = Eigen::Vector3d::Identity() * 0.001;

	//// For Left Arm (0~6)
	Matrix3d Rot_45;
	Rot_45.setIdentity();
	Rot_45(1, 1) = cos(-M_PI / 4.0);
	Rot_45(1, 2) = -sin(-M_PI / 4.0);
	Rot_45(2, 1) = sin(-M_PI / 4.0);
	Rot_45(2, 2) = cos(-M_PI / 4.0);	
	
	axis_[0] = Rot_45 * Eigen::Vector3d::UnitZ();
	axis_[1] = Rot_45 * Eigen::Vector3d::UnitY();
	axis_[2] = Rot_45 * Eigen::Vector3d::UnitZ();
	axis_[3] = Rot_45 * -1.0*Eigen::Vector3d::UnitY();
	axis_[4] = Rot_45 * Eigen::Vector3d::UnitZ();
	axis_[5] = Rot_45 * -1.0*Eigen::Vector3d::UnitY();
	axis_[6] = Rot_45 * Eigen::Vector3d::UnitZ();

	joint_position_global_[0] = Eigen::Vector3d(0.1496, 0.3773, 0.7663);
	joint_position_global_[1] = Eigen::Vector3d(0.1496, 0.3773, 0.7663);
	joint_position_global_[2] = Eigen::Vector3d(0.1496, 0.6008, 0.9898);
	joint_position_global_[3] = Eigen::Vector3d(0.2321, 0.6008, 0.9898);
	joint_position_global_[4] = Eigen::Vector3d(0.1496, 0.8723, 1.2613);
	joint_position_global_[5] = Eigen::Vector3d(0.1496, 0.8723, 1.2613);
	joint_position_global_[6] = Eigen::Vector3d(0.0616, 0.8723, 1.2613);
	
	joint_position_local_[0] = joint_position_global_[0];

	for (int i=1; i < 7; i++)
		joint_position_local_[i] = joint_position_global_[i] - joint_position_global_[i-1];

	com_position_[0] = Vector3d(0.1497, 0.2995, 0.7374);
	com_position_[1] = Vector3d(0.1498, 0.4557, 0.7960);
	com_position_[2] = Vector3d(0.1830, 0.5904, 0.9417);
	com_position_[3] = Vector3d(0.1827, 0.6120, 1.0386);
	com_position_[4] = Vector3d(0.1509, 0.8254, 1.1545);
	com_position_[5] = Vector3d(0.1075, 0.8543, 1.2578);
	com_position_[6] = Vector3d(0.0496, 0.9200, 1.3260);

	for (int i = 0; i < 7; i++)
		com_position_[i] -=  joint_position_global_[i];

	for (int i = 0; i < 7; i++) {
		body_[i] = Body(mass_[i], com_position_[i], inertia_[i]);
		joint_[i] = Joint(JointTypeRevolute, axis_[i]);

		if (i == 0)
			body_id_[i] = model_->AddBody(base_id_, Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	}

	//// For Right Arm (7~13)
	axis_[7] = Rot_45.transpose() * Eigen::Vector3d::UnitZ();
	axis_[8] = Rot_45.transpose() * Eigen::Vector3d::UnitY();
	axis_[9] = Rot_45.transpose() * Eigen::Vector3d::UnitZ();
	axis_[10] = Rot_45.transpose() * -1.0*Eigen::Vector3d::UnitY();
	axis_[11] = Rot_45.transpose() * Eigen::Vector3d::UnitZ();
	axis_[12] = Rot_45.transpose() * -1.0*Eigen::Vector3d::UnitY();
	axis_[13] = Rot_45.transpose() * Eigen::Vector3d::UnitZ();

	joint_position_global_[7] = Eigen::Vector3d(0.1496, -0.3773, 0.7663);
	joint_position_global_[8] = Eigen::Vector3d(0.1496, -0.3773, 0.7663);
	joint_position_global_[9] = Eigen::Vector3d(0.1496, -0.6008, 0.9898);
	joint_position_global_[10] = Eigen::Vector3d(0.2321, -0.6008, 0.9898);
	joint_position_global_[11] = Eigen::Vector3d(0.1496, -0.8723, 1.2613);
	joint_position_global_[12] = Eigen::Vector3d(0.1496, -0.8723, 1.2613);
	joint_position_global_[13] = Eigen::Vector3d(0.0616, -0.8723, 1.2613);

	joint_position_local_[7] = joint_position_global_[7];

	for (int i = 8; i < dof; i++)
		joint_position_local_[i] = joint_position_global_[i] - joint_position_global_[i - 1];

	com_position_[7] = Vector3d(0.1497, -0.3483, 0.6876);
	com_position_[8] = Vector3d(0.1498, -0.4069, 0.8439);
	com_position_[9] = Vector3d(0.1830, -0.5526, 0.9785);
	com_position_[10] = Vector3d(0.1827, -0.6495, 1.0002);
	com_position_[11] = Vector3d(0.1509, -0.7654, 1.2137);
	com_position_[12] = Vector3d(0.1075, -0.8688, 1.2426);
	com_position_[13] = Vector3d(0.0496, -0.9369, 1.3082);

	for (int i = 7; i < dof; i++)
		com_position_[i] -= joint_position_global_[i];

	for (int i = 7; i < dof; i++) {
		body_[i] = Body(mass_[i], com_position_[i], inertia_[i]);
		joint_[i] = Joint(JointTypeRevolute, axis_[i]);

		if (i == 7)
			body_id_[i] = model_->AddBody(base_id_, Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	}
}
void HQP::robot::RobotModel::Jacobian(const int & frame_id) { //?
	MatrixXd J_temp(6, m_na_ + 5);
	J_temp.setZero();
	if (frame_id == 0) {
		CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id ], -1.0*joint_position_local_[frame_id], J_temp, true);
	}
	else {
		CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], J_temp, true);
	}
	//cout << "J_temp" << J_temp.bottomRows(3) << endl;
	m_J_ = J_temp * m_selection_;
	J_temp = m_J_;
	//cout << "J_temp" << J_temp.bottomRows(3) << endl;
	for (int i = 0; i < 2; i++) {
		m_J_.block(i * 3, 0, 3, m_na_ + 2) = J_temp.block(3 - i * 3, 0, 3, m_na_ + 2);
	}
}
void  HQP::robot::RobotModel::JacobianEE(const VectorXd & q) {
	VectorXd q_temp(m_na_ + 5);
	MatrixXd J_temp(6, m_na_ + 5);
	q_temp.setZero();
	
	m_J_EE[0].resize(6, dof / 2);
	m_J_EE[0].setZero();
	q_temp.segment(5, dof / 2) = q.segment(5, dof / 2);
	CalcPointJacobian6D(*model_, q_temp, body_id_[6], com_position_[6], J_temp, true);
	for (int i = 0; i < 2; i++) {
		m_J_EE[0].block(i * 3, 0, 3, dof / 2) = J_temp.block(3 - i * 3, 5, 3, dof / 2);
	}

	m_J_EE[1].resize(6, dof / 2);
	m_J_EE[1].setZero();
	q_temp.setZero();
	q_temp.segment(5 + dof /2, dof / 2) = q.segment(5 + dof / 2, dof / 2);

	CalcPointJacobian6D(*model_, q_temp, body_id_[13], com_position_[13], J_temp, true);
	for (int i = 0; i < 2; i++) {
		m_J_EE[1].block(i * 3, 0, 3, dof / 2) = J_temp.block(3 - i * 3, 5 + dof / 2, 3, dof / 2);
	}
}
void HQP::robot::RobotModel::Position(const int & frame_id) { // for mobile
	m_pos_ = CalcBodyToBaseCoordinates(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], true);
}
void HQP::robot::RobotModel::Orientation(const int & frame_id) { // for mobile
	m_Ori_ = CalcBodyWorldOrientation(*model_, q_rbdl_, body_id_[frame_id - 1], true).transpose();
}
void HQP::robot::RobotModel::Transformation(const int & frame_id) { // for mobile
	Position(frame_id);
	Orientation(frame_id);
	m_Trans_.linear() = m_Ori_;
	m_Trans_.translation() = m_pos_;
}
void HQP::robot::RobotModel::NLEtorque() { // for mobile
	VectorXd NLE_virtual(5 + m_na_);
	MassMatrix();
	InverseDynamics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_, tau_);

	for (int i = 0; i < 5 + m_na_; i++)
		NLE_virtual(i) = tau_(i);

	VectorXd q_real_dot(dof +2);
	q_real_dot.setZero();
	q_real_dot.head(2) = qdot_rbdl_.head(2);

	m_NLE_ = m_selection_.transpose() * NLE_virtual + m_selection_.transpose() * m_Mass_virtual_mat_ * m_selection_dot_ * q_real_dot;
}
void HQP::robot::RobotModel::MassMatrix() { // for mobile
	MatrixXd Mass_virtual(5 + m_na_, 5 + m_na_);
	Mass_virtual.setZero();
	CompositeRigidBodyAlgorithm(*model_, q_rbdl_, m_Mass_virtual_mat_, true);
	m_Mass_mat_ = m_selection_.transpose() * m_Mass_virtual_mat_ * m_selection_;
}
void HQP::robot::RobotModel::Manipulability(const VectorXd &  q) {
	JacobianEE(q);
	m_manipulability_[0] = sqrt( (m_J_EE[0] * m_J_EE[0].transpose()).determinant());
	m_manipulability_[1] = sqrt((m_J_EE[1]* m_J_EE[1].transpose()).determinant());
}
void HQP::robot::RobotModel::ManipulabilityJacobian() {	
	m_J_manipulability_[0].resize(dof / 2);
	m_J_manipulability_[1].resize(dof / 2);
	VectorXd q = q_rbdl_;
	Manipulability(q);
	const double mani_0 = m_manipulability_[0];
	const double mani_1 = m_manipulability_[1];

	const double h = 0.0000001;
	for (int i = 0; i < dof / 2; i++) {
		q(i + 5) += h;
		q(i + 5 + dof / 2) += h;
		Manipulability(q);
		m_J_manipulability_[0](i) = (m_manipulability_[0] - mani_0) / h;
		m_J_manipulability_[1](i) = (m_manipulability_[1] - mani_1) / h;
		q = q_rbdl_;
	}
}
void HQP::robot::RobotModel::getUpdateKinematics(const VectorXd & q, const VectorXd & qdot) { // for mobile
	q_rbdl_ = q;
	if (q(2) < 0.0)
		q_rbdl_(2) = M_PI * 2 + q(2);
	
	qdot_rbdl_ = qdot;	
	qddot_rbdl_.setZero();
	UpdateKinematics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_);

	// for mobile
	m_base_.setIdentity();
	m_base_.rotate(AngleAxisd(q(2), Vector3d::UnitZ()));
	//m_base_.linear() = rot;
	m_base_.translation().head(2) = q.head(2);

	m_mobile_dot_.setZero();
	m_mobile_dot_.linear().head(2) = qdot.head(2);
	m_mobile_dot_.angular()(2) = qdot(2);

	// for selection matrix
	double r = 0.165, b = 0.5, d = 0.05, c = r / (2.0 * b);
	
	m_selection_.bottomRightCorner(m_na_ + 2, m_na_ + 2).setIdentity();
	m_selection_(0, 0) = c*(b*cos(q_rbdl_(2)) - d*sin(q_rbdl_(2)));
	m_selection_(0, 1) = c*(b*cos(q_rbdl_(2)) + d*sin(q_rbdl_(2)));
	m_selection_(1, 0) = c*(b*sin(q_rbdl_(2)) + d*cos(q_rbdl_(2)));
	m_selection_(1, 1) = c*(b*sin(q_rbdl_(2)) - d*cos(q_rbdl_(2)));

	m_selection_(2, 0) = -c;
	m_selection_(2, 1) = c;
	m_selection_(3, 0) = 1.0;
	m_selection_(4, 1) = 1.0;

	m_selection_dot_(0, 0) = c*(-b*sin(q_rbdl_(2)) - d*cos(q_rbdl_(2)));
	m_selection_dot_(0, 1) = c*(-b*sin(q_rbdl_(2)) + d*cos(q_rbdl_(2)));
	m_selection_dot_(1, 0) = c*(b*cos(q_rbdl_(2)) - d*sin(q_rbdl_(2)));
	m_selection_dot_(1, 1) = c*(b*cos(q_rbdl_(2)) + d*sin(q_rbdl_(2)));

	m_selection_dot_ *= qdot(2);
	
}

void HQP::robot::RobotModel::PointVelocity(const int & frame_id) {
	p_dot_ = CalcPointVelocity6D(*model_, q_rbdl_, qdot_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1]);
	m_p_dot_.angular() = p_dot_.head<3>();
	m_p_dot_.linear() = p_dot_.tail<3>();
}

