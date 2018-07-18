#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// #include "ethercat_elmo.h"
#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"
#include "haptic.h"
#include "robostar_7_dof_robot_hw_config.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;


static double staright(double time,     ///< Current time
	double time_0,   ///< Start time
	double time_f,   ///< End time
	double x_0,      ///< Start state
	double x_f      ///< End state
);


class ArmController
{
public:
    enum ARM_CONTROL_MODE
    {
		NONE,
		HOME_JOINT_CTRL,
		INIT_JOINT_CTRL,
		HOME_TORQUE_CTRL,
		SIMPLE_JACOBIAN,
		FEEDBACK_JACOBIAN,
		CLIK,
		CLIK_WITH_WEIGHTED_PSEUDO_INV,
		SIMPLE_PD,
		SIMPLE_PD_WITH_GRAVITY_COMP_STEP,
		SIMPLE_PD_WITH_GRAVITY_COMP_CUBIC,
		SIMPLE_PD_WITH_GRAVITY_COMP_STRAIGHT,
		PD_WITH_DYNAMIC_COMP_STEP,
		PD_WITH_DYNAMIC_COMP_CUBIC,
		PD_WITH_DYNAMIC_COMP_STRAIGHT,
		TASK_PD_WITH_GRAVITY_COMP_STEP,
		TASK_PD_WITH_GRAVITY_COMP_CUBIC,
		TASK_PD_WITH_DYNAMIC_COMP_STEP,
		TASK_PD_WITH_DYNAMIC_COMP_CUBIC,
		GRAVITY_COMP,
		SCA_Weighting,
		SCA_Task_Transition,
		SCA_Adding_Two_Tasks


		// TODO: implement your own mode
		
    };

private:
    size_t dof_;

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d torque_desired_; // Control value

	// Task space
	Vector3d x_init_;
	Vector3d x_;
	Matrix3d rotation_;
	Matrix3d Rot[6];
	Matrix3d rotation_init_;
	Vector3d phi_;
	Vector6d x_dot_; // 6D (linear + angular)
	Vector6d x_error_; 
	Vector3d x_dot_linear;
	Vector3d x_dot_angular;
	// Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d A_; // Mass matrix
	Matrix7d A_inverse; // Inverse of mass matrix

	// For controller
	Matrix<double,3,7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage 
	
	VectorXd q_temp_;	// For RBDL bug
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd j_temp_;	// For RBDL bug
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL bug



	Vector7d q_cubic_;
	Vector7d q_target_;

	Vector3d x_cubic_;
	Vector3d x_target_;

    unsigned long tick_;
    double play_time_;
    double hz_;
    double control_start_time_;

    ARM_CONTROL_MODE control_mode_;
    bool is_mode_changed_;

	// self collision avoidance
	Vector7d dd_dq[10];
	double dP_dd[10];
	Vector7d dP_dq[10];
	Vector7d dP_dq_prev[10];
	Vector7d dP_dq_del[10];
	MatrixXd W_col[10];
	MatrixXd W_total;
	Vector6d x_task;
	Vector3d x_pos;
	Vector3d x_ori;

	Vector3d x_avoid[4];
	VectorXd dis_vector;
	int min_index;

	MatrixXd Jaco[6];
	MatrixXd Jaco_[6];
	MatrixXd Jaco_pos[6];

	double alpha,beta,phi;
	Vector3d x_desired;
	Vector3d x_dot_desired;

	// for robot model construction
    Math::Vector3d com_pos_[DOF];
    Vector3d joint_pos_[DOF];
    Math::Vector3d com_pos_sub[DOF];
    Vector3d joint_pos_sub[DOF];
	Math::Vector3d com_pos_dis[4];

    double mass[DOF];
    Vector3d axis[DOF];
    Math::Vector3d inertia[DOF];


    shared_ptr<Model> model_;
	shared_ptr<Model> model_sub;

    unsigned int body_id_[DOF];
	unsigned int body_id_sub[DOF];


    Body body_[DOF];
	Body body_sub[DOF];

	Body body_1[DOF];
    Body body_2[DOF];
    Body body_3[DOF];
    Body body_4[DOF];
    Body body_5[DOF];
    Body body_6[DOF];

    Joint joint_[DOF];

	Matrix7d Iden7;

	struct Task
	{
		MatrixXd J, N, NT;
		MatrixXd J_inv;
		MatrixXd Jdot;
		VectorXd x_, xd, xdot, xdot_d, xddot;
		VectorXd x_cubic, xdot_cubic, xddot_cubic;
		VectorXd T;
		MatrixXd Lambda_inv, Lambda;
		MatrixXd Mu;
		MatrixXd J_bar_T;
		Vector3d delphi;
		float h;
		VectorXd T_star;
	};
	Task _task1, _task2;

	// for hw5



private:
	void initFileDebug();
    void printState();
	void moveJointPosition(const Vector7d &target_pos, double duration);

public:
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque);
	void readData(const Vector7d &position, const Vector7d &velocity);
	void writeData(Vector7d &position, Vector7d &velocity, Vector7d &torque);
	void writePositionData(VectorXd &position);
	void writeTorqueData(VectorXd &torque);

	double dis[10];
	Vector3d link_com[7];
	Vector3d joint_pos[7];
	Matrix3d link_rot[7];
	Vector3d P_a[10];
	Vector3d P_b[10];
	Vector3d target_pos;
	VectorXd dist_vector;

public:
		ArmController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_(NONE), is_mode_changed_(false)
	{
        initDimension(); initModel(); initFileDebug();
	}


	double Get_h_value(double distance);
    void setMode(ARM_CONTROL_MODE mode);
    void initDimension();
    void initModel();
    void initPosition();
    void compute();
};

#endif
