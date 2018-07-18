#include "include/robot/controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>

FILE *fp1 = fopen("SCA_data_weighting_20cm","w");
FILE *fp2 = fopen("SCA_data_reference_20cm","w");
FILE *fp3 = fopen("SCA_data_task_transition_20cm","w");

static double straight(double time,     ///< Current time
	double time_0,   ///< Start time
	double time_f,   ///< End time
	double x_0,      ///< Start state
	double x_f      ///< End state
)
{
	double x_t;

	if (time < time_0)
	{
		x_t = x_0;
	}
	else if (time > time_f)
	{
		x_t = x_f;
	}
	else
	{
		x_t = (x_f - x_0) / (time_f - time_0) * (time - time_0) + x_0;
	}

	return x_t;
}

void ArmController::compute()
{
	const double ON_THE_RECORD = 1.0;
	// Kinematics calculation ------------------------------

	q_temp_ = q_;
	qdot_temp_ = qdot_;

	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, &qdot_temp_, NULL);


	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_pos_[DOF - 1], true);
	x_dot_ = CalcPointVelocity6D(*model_, q_, qdot_, body_id_[DOF -1], com_pos_[DOF -1], true);
	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_pos_[DOF - 1], j_temp_, true);
	for(int i=0;i<3;i++)
	{
		x_dot_linear(i) = x_dot_(i+3);
		x_dot_angular(i) = x_dot_(i);
	}

	for (int i=0;i<DOF;i++)
	{
	 link_com[DOF-1-i] = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF-1-i], com_pos_[DOF-1-i], true);
	 link_rot[DOF-1-i] = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF-1-i], true).transpose();
	 joint_pos[DOF-1-i] = CalcBodyToBaseCoordinates(*model_sub, q_, body_id_sub[DOF-1-i], com_pos_sub[DOF-1-i], true); 
	}

	com_pos_dis[0] = joint_pos_[5]*(P_a[0]-joint_pos[4]).dot(joint_pos[5]-joint_pos[4])/(joint_pos[5] - joint_pos[4]).norm()/(joint_pos[5] - joint_pos[4]).norm()  ;
	com_pos_dis[1] = joint_pos_[6]*(P_a[1]-joint_pos[5]).dot(joint_pos[6]-joint_pos[5])/(joint_pos[6]-joint_pos[5]).norm()/(joint_pos[6]-joint_pos[5]).norm() ;
	com_pos_dis[2] = joint_pos_[5]*(P_a[2]-joint_pos[4]).dot(joint_pos[5]-joint_pos[4])/(joint_pos[5] - joint_pos[4]).norm()/(joint_pos[5] - joint_pos[4]).norm()  ;
	com_pos_dis[3] = joint_pos_[6]*(P_a[3]-joint_pos[5]).dot(joint_pos[6]-joint_pos[5])/(joint_pos[6]-joint_pos[5]).norm()/(joint_pos[6]-joint_pos[5]).norm() ;

	CalcPointJacobian6D(*model_, q_, body_id_[4], com_pos_dis[0], Jaco[0], true);
	CalcPointJacobian6D(*model_, q_, body_id_[5], com_pos_dis[1], Jaco[1], true);
	CalcPointJacobian6D(*model_, q_, body_id_[4], com_pos_dis[2], Jaco[2], true);
	CalcPointJacobian6D(*model_, q_, body_id_[5], com_pos_dis[3], Jaco[3], true);

	// cout << "2" << j_temp_ <<"\n" <<endl;

	// cout << "1" << com_pos_dis[1] << endl;
	// cout << "2" << com_pos_[5] << endl;
	// cout << "jaco" << Jaco[1] << endl;	

	NonlinearEffects(*model_, q_, Vector7d::Zero(), g_temp_);
	CompositeRigidBodyAlgorithm(*model_, q_, m_temp_, true);

	g_= g_temp_;
	A_ = m_temp_;
	A_inverse = A_.inverse();


	for (int i = 0; i<2; i++){
		j_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);

		for(int j=0;j<4;j++)
		Jaco_[j].block<3, DOF>(i * 3, 0) = Jaco[j].block<3, DOF>(3 - i * 3, 0);

	}
	// -----------------------------------------------------
	j_v_ = j_.block < 3, DOF>(0, 0);

	for(int i=0;i<6;i++)
	Jaco_pos[i] = Jaco_[i].block < 3, DOF>(0, 0);

	//cout << "Calc Jaco" << Jaco_pos[1] << endl;

	// ---------------------------------
	//
	// q_		: joint position
	// qdot_	: joint velocity
	// x_		: end-effector position 
	// j_		: end-effector basic jacobian
	// m_		: mass matrix
	//
	//-------------------------------------------------------------------
	if (is_mode_changed_)
	{
		is_mode_changed_ = false;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();

		x_init_ = x_;
		rotation_init_ = rotation_;
	}


	switch (control_mode_)
	{
	case NONE:
		break;
	case HOME_JOINT_CTRL:
	{
		// The code for the 'h' command
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

		moveJointPosition(target_position, 2.0);
		break;
	}
	case INIT_JOINT_CTRL:
	{
		// The code for the 'i' command

		q_desired_ << 0.0, -M_PI/4. , 0.0, M_PI / 2., 0.0, M_PI/4., 0.0;
		Matrix7d kp, kv;
		kp = 20.0 * Matrix7d::Identity();
		kv = 0.1 * Matrix7d::Identity();
		torque_desired_ = kp * (q_desired_ - q_) + kv * (-qdot_) + g_;
			

		break;
	}
	case SIMPLE_JACOBIAN:
	{
		Vector3d x_desired;
		Vector3d x_dot_desired;
		double duration = 2.0;
		x_desired << 0.1, 0.2, 0.6;
		Matrix<double, 7, 3> j_v_inverse;
		j_v_inverse = j_v_.transpose() * (j_v_ * j_v_.transpose()).inverse();

		for (int i = 0; i < 3; i++)
		{
			x_dot_desired(i) = DyrosMath::cubicDot(play_time_,
				control_start_time_, control_start_time_ + duration,
				x_init_(i), x_desired(i), 0, 0, hz_);
		}
		//cout << x_dot_desired << endl;
		q_desired_ = q_desired_ + j_v_inverse * x_dot_desired / hz_;

		if (play_time_ <= control_start_time_ + ON_THE_RECORD)
		{
			// for (int i = 0; i < 7; i++)
			// {
			// 	simple_jacobian_file_ << q_(i) << "\t";
			// }
			// for (int i = 0; i < 3; i++)
			// {
			// 	simple_jacobian_file_ << x_(i) << "\t";
			// }
			// simple_jacobian_file_ << endl;
		}
		break;
	}
	case FEEDBACK_JACOBIAN:
	{
		Vector3d x_desired;
		double duration = 2.0;
		x_desired << 0.1, 0.2, 0.6;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		Matrix<double, 7, 3> j_v_inverse;
		j_v_inverse = j_v_.transpose() * (j_v_ * j_v_.transpose()).inverse();
		q_desired_ = q_ + j_v_inverse * (x_cubic_ - x_);
		if (play_time_ <= control_start_time_ + ON_THE_RECORD)
		{
			// for (int i = 0; i < 7; i++)
			// {
			// 	feedback_jacobian_file_ << q_(i) << "\t";
			// }
			// for (int i = 0; i < 3; i++)
			// {
			// 	feedback_jacobian_file_ << x_(i) << "\t";
			// }
			// feedback_jacobian_file_ << endl;
		}
		break;
	}
	case CLIK:
	{
		Vector3d x_desired;
		Vector3d x_dot_desired;
		Matrix3d kp;
		kp = Matrix3d::Identity() * 5;

		double duration = 2.0;
		x_desired << 0.1, 0.2, 0.6;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		for (int i = 0; i < 3; i++)
		{
			x_dot_desired(i) = DyrosMath::cubicDot(play_time_,
				control_start_time_, control_start_time_ + duration,
				x_init_(i), x_desired(i), 0, 0, hz_);
		}
		Matrix<double, 7, 3> j_v_inverse;
		j_v_inverse = j_v_.transpose() * (j_v_ * j_v_.transpose()).inverse();
		q_desired_ = q_desired_ + j_v_inverse * (x_dot_desired + kp * (x_cubic_ - x_)) / hz_;
		if (play_time_ <= control_start_time_ + ON_THE_RECORD)
		{
			// for (int i = 0; i < 7; i++)
			// {
			// 	clik_file_ << q_(i) << "\t";
			// }
			// for (int i = 0; i < 3; i++)
			// {
			// 	clik_file_ << x_(i) << "\t";
			// }
			// clik_file_ << endl;
		}
		break;
	}
	case CLIK_WITH_WEIGHTED_PSEUDO_INV:
	{
		Vector3d x_desired;
		Vector3d x_dot_desired;
		Matrix7d w;
		Matrix3d kp;
		kp = Matrix3d::Identity() * 5;
		w = Matrix7d::Identity();
		w(3, 3) = 0.001;

		double duration = 2.0;
		x_desired << 0.1, 0.2, 0.6;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		for (int i = 0; i < 3; i++)
		{
			x_dot_desired(i) = DyrosMath::cubicDot(play_time_,
				control_start_time_, control_start_time_ + duration,
				x_init_(i), x_desired(i), 0, 0, hz_);
		}
		Matrix<double, 7, 3> j_v_inverse;
		j_v_inverse = w * j_v_.transpose() * (j_v_ * w * j_v_.transpose()).inverse();
		q_desired_ = q_desired_ + j_v_inverse * (x_dot_desired + kp * (x_cubic_ - x_)) / hz_;
		if (play_time_ <= control_start_time_ + ON_THE_RECORD)
		{
			// for (int i = 0; i < 7; i++)
			// {
			// 	clik_weight_file_ << q_(i) << "\t";
			// }
			// for (int i = 0; i < 3; i++)
			// {
			// 	clik_weight_file_ << x_(i) << "\t";
			// }
			// clik_weight_file_ << endl;
		}
		break;
	}
	case HOME_TORQUE_CTRL:
	{
		double duration = 5.0;
		Vector7d target_pos;
		target_pos.setZero();

		q_desired_ = DyrosMath::cubicVector<7>(play_time_,
			control_start_time_,
			control_start_time_ + duration, q_init_, target_pos, Vector7d::Zero(), Vector7d::Zero());
		Matrix7d kp, kv;

		kp = 1000.0 * Matrix7d::Identity();
		kv = 10.0 * Matrix7d::Identity();

		torque_desired_ = A_ * (kp * (q_desired_ - q_) + kv * (-qdot_)) + g_;
		break;
	}
	case GRAVITY_COMP:
		torque_desired_ = g_;
		break;
	case SCA_Weighting:
	{
		phi = 100.0;
		alpha = 50.0;
		beta = 2.0;
		double col_pair = 4.0;
		W_total.setZero();

		 // Weighting Matrix Setting
		for(int j=0;j<col_pair;j++)
		{
		dP_dd[j] = -phi*exp(-alpha*dis[j])*pow(dis[j],-1.0*beta)*(beta/dis[j] + alpha);
		dd_dq[j] = 1/dis[j]*Jaco_pos[j].transpose()*(P_a[j] - P_b[j]);
		dP_dq[j] = dP_dd[j]*dd_dq[j];
		dP_dq_del[j] = dP_dq[j] - dP_dq_prev[j];
		dis_vector(j) = dis[j];

			for(int i=0;i<DOF;i++)
			{
				if( abs(dP_dq_del[j](i)) >= 0 )
				W_col[j](i, i) = 1.0 + abs(dP_dq[j](i));

				else
				W_col[j](i, i) = 1.0;

			}
		W_total +=  W_col[j];
		dP_dq_prev[j] = dP_dq[j];	
		}
		W_total = W_total/col_pair;

		// Task Setting
		double duration = 3.0;
		x_desired << -0.20, 0.0, 0.0;
		x_desired = x_init_ + x_desired;
		//cout << x_desired << endl;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		for (int i = 0; i < 3; i++)
		{
			x_dot_desired(i) = DyrosMath::cubicDot(play_time_,
				control_start_time_, control_start_time_ + duration,
				x_init_(i), x_desired(i), 0, 0, hz_);
		}
		x_pos =  400.0*(x_cubic_ - x_) - 20.0*x_dot_linear;
		target_pos = x_cubic_;
		//cout << x_cubic_ << endl;
		 //cout << "error" <<dP_dq[1] << "\n" << endl;
		 //cout << "2" <<W_col[1] << "\n" << endl;
		// cout << "3" <<W_col[2] << "\n" << endl;
		// cout << "4" <<W_col[3] << "\n" << endl;
		 double index;
		 dis_vector.minCoeff(&index);

		//  cout << "min index" << index << endl;
		//  cout << dis[0] << "\t" << dis[1] << "\t" << dis[2] <<"\t" << dis[3] << endl;

		x_ori = - 10.0* DyrosMath::getPhi(rotation_, rotation_init_) - x_dot_angular;

		for(int i=0; i < 3; i++)
		{
			x_task(i) = x_pos(i);
			x_task(i+3) = x_ori(i);
		}

		torque_desired_ = W_total.inverse()* j_v_.transpose()*x_pos + g_;

	 fprintf(fp1, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t\n", dis[0], dis[1], dis[2], dis[3], x_cubic_(0)- x_(0), x_cubic_(1)- x_(1), x_cubic_(2)- x_(2), x_ori.norm());


		break;
	}
	case SCA_Task_Transition : 
	{
		for(int i=0;i<4;i++)
		{
			x_avoid[i] = (P_a[i] - P_b[i])/(P_a[i] - P_b[i]).norm() ;
			dis_vector(i) = dis[i];
		}

		// 가장 가까운 거리에 대한 자코비안과 멀어지는 방향 벡터 구하기
		// 1. 가장 가까운 거리에 대한 인덱스 구하기
		dis_vector.minCoeff(&min_index);

		// Task Transition Algorithm
		// 4번 관절이 접히는 방향의 자코비안이 1번 작업의 자코비안이 되기 때문에
		// 2번 작업은 4번 관절이 접히는 방향으로 거의 움직이지 못함
		// 2번 작업은 관절 2,5,6번으로만 움직이려고 함

		_task1.J = x_avoid[min_index].transpose()*Jaco_pos[min_index]; // 1 X 7
		_task1.Lambda_inv = _task1.J*A_inverse*_task1.J.transpose(); // 1 X 1
		_task1.Lambda = _task1.Lambda_inv.inverse();
		_task1.J_bar_T = (A_inverse*_task1.J.transpose()*_task1.Lambda).transpose(); // 1 X 7

		_task1.T_star = _task2.T;		
		_task1.h = Get_h_value(dis[min_index]);
		_task1.xdot = _task1.J*qdot_;
		_task1.x_ = _task1.h*x_avoid[min_index].transpose()*(_task1.h*(10.0*x_avoid[min_index]) + (1.0 - _task1.h)*_task1.J*A_inverse*_task1.T_star); // 1-D

		_task1.T = _task1.J.transpose()*_task1.Lambda*_task1.x_; // 7 X 1
		_task1.NT = Iden7 - _task1.J.transpose()*_task1.J_bar_T;
		_task1.N = _task1.NT.transpose();

		_task2.J = j_v_*_task1.N;
		_task2.Lambda_inv = _task2.J*A_inverse*_task2.J.transpose();
		_task2.Lambda = _task2.Lambda_inv.inverse();

		// Task Setting
		double duration = 3.0;
		x_desired << -0.20, 0.0, 0.0;
		x_desired = x_init_ + x_desired;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,control_start_time_, control_start_time_ + duration,x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		for (int i = 0; i < 3; i++)
		{
			x_dot_desired(i) = DyrosMath::cubicDot(play_time_,control_start_time_, control_start_time_ + duration,x_init_(i), x_desired(i), 0, 0, hz_);
		}
		x_pos =  400.0*(x_cubic_ - x_) - 20.0*x_dot_linear;
		target_pos = x_cubic_;
		_task2.h = 1.0 ;
		_task2.x_ = _task2.h*x_pos ;

		_task2.T = _task2.J.transpose()*_task2.Lambda*_task2.x_;

		 fprintf(fp3, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t \n", dis[0], dis[1], dis[2], dis[3], x_cubic_(0)- x_(0), x_cubic_(1)- x_(1), x_cubic_(2)- x_(2), _task1.h);


		torque_desired_ = _task1.T + _task2.T + g_;

		break;
	}
		case SCA_Adding_Two_Tasks : 
	{
		for(int i=0;i<4;i++)
		{
			x_avoid[i] = (P_a[i] - P_b[i])/(P_a[i] - P_b[i]).norm() ;
			dis_vector(i) = dis[i];
		}

		// 가장 가까운 거리에 대한 자코비안과 멀어지는 방향 벡터 구하기
		// 1. 가장 가까운 거리에 대한 인덱스 구하기
		dis_vector.minCoeff(&min_index);

		//	cout << _task1.J  << endl;
		// Task Transition Algorithm
		_task1.J = Jaco_pos[min_index];
		_task1.Lambda_inv = _task1.J*A_inverse*_task1.J.transpose();
		_task1.Lambda = _task1.Lambda_inv.inverse();
		_task1.J_bar_T = A_inverse*_task1.J.transpose()*_task1.Lambda;

		_task1.h = Get_h_value(dis[min_index]);
		_task1.xdot = _task1.J*qdot_;
		_task1.x_ = (_task1.h*(3.0*x_avoid[min_index])) ; // - _task1.xdot

		_task1.T = _task1.J.transpose()*_task1.Lambda*_task1.x_;

		_task2.J = j_v_;
		_task2.Lambda_inv = _task2.J*A_inverse*_task2.J.transpose();
		_task2.Lambda = _task2.Lambda_inv.inverse();

		// Task Setting
		double duration = 3.0;
		x_desired << -0.20, 0.0, 0.0;
		x_desired = x_init_ + x_desired;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		for (int i = 0; i < 3; i++)
		{
			x_dot_desired(i) = DyrosMath::cubicDot(play_time_,control_start_time_, control_start_time_ + duration,x_init_(i), x_desired(i), 0, 0, hz_);
		}
		x_pos =  400.0*(x_cubic_ - x_) - 20.0*x_dot_linear;
		target_pos = x_cubic_;
		_task2.x_ = x_pos ;
		//+ (1.0 - _task2.h)*_task2.J*A_inverse*_task1.T;
		//_task2.h = 1.0 - _task1.h;

		_task2.T = _task2.J.transpose()*_task2.Lambda*_task2.x_;

		// cout << "task1" << _task1.J << endl;
		// cout << "task2" << _task2.J << endl;
		// cout <<"jv" << j_v_ << endl;
		//cout << _task1.h << endl;

	 fprintf(fp2, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t \n", dis[0], dis[1], dis[2], dis[3], x_cubic_(0)- x_(0), x_cubic_(1)- x_(1), x_cubic_(2)- x_(2));


		torque_desired_ =   _task2.T + g_;

		break;
	}
	default:
		break;
	}

	printState();

	tick_++;
	play_time_ = tick_ / hz_;	// second
}

void ArmController::setMode(ARM_CONTROL_MODE mode)
{
    is_mode_changed_ = true;
    control_mode_ = mode;
    cout << "Current mode (changed) : " ;

    switch (control_mode_)
    {

	case NONE:
		cout << "NONE";
		break;
	case HOME_JOINT_CTRL:
		cout << "HOME_JOINT_CTRL";
		break;
	case INIT_JOINT_CTRL:
		cout << "INIT_JOINT_CTRL";
		break;
	case SIMPLE_JACOBIAN:
		cout << "SIMPLE_JACOBIAN";
		break;
	case FEEDBACK_JACOBIAN:
		cout << "FEEDBACK_JACOBIAN";
		break;
	case CLIK:
		cout << "CLIK";
		break;
	case CLIK_WITH_WEIGHTED_PSEUDO_INV:
		cout << "CLIK_WITH_WEIGHTED_PSEUDO_INV";
		break;
	case SCA_Weighting:
		cout << "Self-collision Avoidance - Weighting Matrix";
		break;
	case SCA_Task_Transition:
		cout << "Self-collision Avoidance - Task Transition";
		break;
	case SCA_Adding_Two_Tasks:
		cout << "Self-collision Avoidance - Adding two tasks";
	default:
		cout << "???";
		break;
	}
    cout << endl;
}


void ArmController::initFileDebug()
{
	// simple_jacobian_file_.open("simple_jacobian.txt");
	// feedback_jacobian_file_.open("feedback_jacobian.txt");
	// clik_file_.open("clik.txt");
	// clik_weight_file_.open("clik_weight.txt");

	// // for hw6
	// torque_joint_pd_file_.open("torque_joint_pd_file.txt");
	// torque_joint_pd_with_gravity_comp_step_file_.open("torque_joint_pd_with_gravity_comp_step_file.txt");
	// torque_joint_pd_with_gravity_comp_cubic_file_.open("torque_joint_pd_with_gravity_comp_cubic_file.txt");
	// torque_joint_pd_with_gravity_comp_straight_file_.open("torque_joint_pd_with_gravity_comp_straight_file.txt");
	// torque_joint_pd_with_dynamic_comp_step_file_.open("torque_joint_pd_with_dynamic_comp_step_file_.txt");
	// torque_joint_pd_with_dynamic_comp_cubic_file_.open("torque_joint_pd_with_dynamic_comp_cubic_file_.txt");
	// torque_joint_pd_with_dynamic_comp_straight_file_.open("torque_joint_pd_with_dynamic_comp_straight_file_.txt");

	// // for hw7
	// torque_task_pd_with_gravity_comp_step_file_.open("torque_task_pd_with_gravity_comp_step_file.txt");
	// torque_task_pd_with_gravity_comp_cubic_file_.open("torque_task_pd_with_gravity_comp_cubic_file.txt");
	// torque_task_pd_with_dynamic_comp_step_file_.open("torque_task_pd_with_dynamic_comp_step_file.txt");
	// torque_task_pd_with_dynamic_comp_cubic_file_.open("torque_task_pd_with_dynamic_comp_cubic_file.txt");
}

void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 20.)
	{
		DBG_CNT = 0;

		// cout << "q now:\t";
		// for (int i = 0; i<dof_; i++)
		// {
		// 	cout << std::fixed << std::setprecision(3) << q_(i) << '\t';
		// }
		// cout << endl;

		// cout << "q desired:\t";
		// for (int i = 0; i<dof_; i++)
		// {
		// 	cout << std::fixed << std::setprecision(3) << q_desired_(i) << '\t';
		// }
		// cout << endl;

		// cout << "t:\t";
		// for (int i = 0; i<dof_; i++)
		// {
		// 	cout << std::fixed << std::setprecision(3) << torque_desired_(i) << '\t';
		// }
		// cout << endl;
		cout << "Error vector \n" ;
		cout << x_cubic_ - x_ << endl;

		//cout << " [Torque ] \n";
		// cout << W_total.inverse()*j_.transpose()*x_task << endl;
		cout << "[H value] \n";
		cout << _task1.h << endl;

		cout << "distance \n";
		cout << dis[min_index] << endl;
	//	cout << " Weighting Matrix  \n ";
	//	cout << W_total << endl;
	}
}

// Controller Core Methods ----------------------------


void ArmController::moveJointPosition(const Vector7d &target_pos, double duration)
{
	Vector7d zero_vector;
	zero_vector.setZero();
	q_desired_ = DyrosMath::cubicVector<7>(play_time_,
		control_start_time_,
		control_start_time_ + duration, q_init_, target_pos, zero_vector, zero_vector);
}

void ArmController::initDimension()
{
	dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);

	qddot_.setZero();

	x_target_.setZero();
	q_desired_.setZero();
	torque_desired_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);

	for(int i=0;i<10;i++){
		W_col[i].resize(DOF,DOF);
		W_col[i].setZero();
		W_col[i].setIdentity();
		dd_dq[i].setZero();
		dP_dq[i].setZero();
		dP_dq_del[i].setZero();
		dP_dq_prev[i].setZero();
	}

	for(int i=0;i<6;i++)
	{
		Jaco[i].resize(6,DOF);
		Jaco[i].setZero();
		Jaco_[i].resize(6, DOF);
		Jaco_[i].setZero();
		Jaco_pos[i].resize(3, DOF);
		Jaco_pos[i].setZero();
	}
	for (int i=0;i<4;i++)
		x_avoid[i].setZero();


	W_total.resize(DOF,DOF);
	W_total.setZero();
	x_task.setZero();
	x_ori.setZero();
	x_pos.setZero();

	dis_vector.resize(4);
	dis_vector.setZero();

	// Task Transtiion
	_task1.J.resize(3, DOF);
	_task1.J.setZero();
	_task1.Lambda.resize(3,3);
	_task1.Lambda.setZero();
	_task1.Lambda_inv.resize(3,3);
	_task1.Lambda_inv.setZero();
	_task1.x_.resize(3);
	_task1.x_.setZero();
	_task1.h = 0.0;
	_task1.T.resize(DOF);
	_task1.T.setZero();
	_task1.xdot.resize(3);
	_task1.xdot.setZero();

	_task1.N.resize(DOF,DOF);
	_task1.N.setZero();
	_task1.NT.resize(DOF,DOF);
	_task1.NT.setZero();
	_task1.J_bar_T.resize(DOF, 3);
	_task1.J_bar_T.setZero();
	_task1.T_star.resize(DOF);
	_task1.T_star.setZero();


	_task2.J.resize(3, DOF);
	_task2.J.setZero();
	_task2.Lambda.resize(3,3);
	_task2.Lambda.setZero();
	_task2.Lambda_inv.resize(3,3);
	_task2.Lambda_inv.setZero();
	_task2.x_.resize(3);
	_task2.x_.setZero();
	_task2.h = 0.0;
	_task2.T.resize(DOF);
	_task2.T.setZero();

	Iden7.setIdentity();

	dist_vector.resize(4);
	dist_vector.setZero();
	




}

void ArmController::initModel()
{
    model_ = make_shared<Model>();
	model_sub = make_shared<Model>();

    model_->gravity = Vector3d(0., 0, -GRAVITY);
    model_sub->gravity = Vector3d(0., 0, -GRAVITY);



    mass[0] = 6.98;
    mass[1] = 1.0;
    mass[2] = 5.23;
    mass[3] = 6.99;
    mass[4] = 3.3;
    mass[5] = 2.59;
    mass[6] = 1.0;

    axis[0] = Vector3d(0, 0, -1);
    axis[1] = Vector3d(0, -1, 0);
    axis[2] = Vector3d(0, 0, -1);
    axis[3] = Vector3d(0, 1, 0);
    axis[4] = Vector3d(0, 0, -1);
    axis[5] = Vector3d(0, -1, 0);
    axis[6] = Vector3d(0, 0, -1);


    inertia[0] = Vector3d(0.02854672, 0.02411810, 0.01684034);
    inertia[1] = Vector3d(0.00262692, 0.00281948, 0.00214297);
    inertia[2] = Vector3d(0.04197161, 0.00856546, 0.04186745);
    inertia[3] = Vector3d(0.04906429, 0.03081099, 0.02803779);
    inertia[4] = Vector3d(0.00935279, 0.00485657, 0.00838836);
    inertia[5] = Vector3d(0.00684717, 0.00659219, 0.00323356);
    inertia[6] = Vector3d(0.00200000, 0.00200000, 0.00200000);

    joint_pos_[0] = Vector3d(0.0, 0.0, 0.0);
    joint_pos_[1] = Vector3d(0.0, 0.0, 0.223) - joint_pos_[0];
    joint_pos_[2] = Vector3d(0.0, 0.118, 0.223) - joint_pos_[1] - joint_pos_[0];
    joint_pos_[3] = Vector3d(0.0, 0.118, 0.5634) - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    joint_pos_[4] = Vector3d(0.0, 0.0, 0.5634) - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    joint_pos_[5] = Vector3d(0.0, 0.0, 0.8634) - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    joint_pos_[6] = Vector3d(0.0, 0.112, 0.8634) - joint_pos_[5] - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];

	com_pos_[0] = Vector3d(-0.00006, -0.0081, 0.1740) - joint_pos_[0];
    com_pos_[1] = Vector3d(0.00007, 0.1259, 0.2706) - joint_pos_[1] - joint_pos_[0];
    com_pos_[2] = Vector3d(0.00043, 0.1259, 0.5158) - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_[3] = Vector3d(-0.00008, -0.0081, 0.6009) - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_[4] = Vector3d(0.0, -0.0096, 0.8259) - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_[5] = Vector3d(0.00002, 0.1214, 0.8759) - joint_pos_[5] - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_[6] = Vector3d(-0.0, 0.112, 0.9690) - joint_pos_[6] - joint_pos_[5] - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
	
	com_pos_sub[0] = Vector3d(0.0, 0.0, 0.0) - joint_pos_[0];
    com_pos_sub[1] = Vector3d(0.0, 0.0, 0.223) - joint_pos_[1] - joint_pos_[0];
    com_pos_sub[2] = Vector3d(0.0, 0.118, 0.223) - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_sub[3] = Vector3d(0.0, 0.118, 0.5634) - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_sub[4] = Vector3d(0.0, 0.0, 0.5634) - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_sub[5] = Vector3d(0.0, 0.0, 0.8634) - joint_pos_[5] - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_sub[6] = Vector3d(0.0, 0.112, 0.8634) - joint_pos_[6] - joint_pos_[5] - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];

    for (int i = 0; i < DOF; i++) {
        body_[i] = Body(mass[i], com_pos_[i], inertia[i]);
        body_sub[i] = Body(mass[i], com_pos_sub[i], inertia[i]);

        joint_[i] = Joint(JointTypeRevolute, axis[i]);
        if (i == 0){
            body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_pos_[i]), joint_[i], body_[i]);
		    body_id_sub[i] = model_sub->AddBody(0, Math::Xtrans(joint_pos_[i]), joint_[i], body_sub[i]);
		}
        else{
            body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_pos_[i]), joint_[i], body_[i]);
			body_id_sub[i] = model_sub->AddBody(body_id_sub[i - 1], Math::Xtrans(joint_pos_[i]), joint_[i], body_sub[i]);
		}
    }
	//


}


void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = torque(i);
	}
}
void ArmController::readData(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}

void ArmController::writeData(Vector7d &position, Vector7d &velocity, Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		position(i) = q_desired_(i);
	}

}
void ArmController::writePositionData(VectorXd &position)
{
	for (size_t i = 0; i < dof_; i++)
	{
		position(i) = q_desired_(i);
	}
}
void ArmController::writeTorqueData(VectorXd &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		torque(i) = torque_desired_(i);
	}
}
void ArmController::initPosition()
{
    q_init_ = q_;
    q_desired_ = q_init_;
}

double ArmController::Get_h_value(double distance)
{
	double h;
	double upper_lim = 0.15;
	double lower_lim = 0.05;

	if(distance < lower_lim)
	{
		h = 1.0;
	}
	else if(lower_lim <= distance && distance < upper_lim)
	{
		h = -1.0/(upper_lim - lower_lim)*(distance - lower_lim) + 1.0;
	}
	else{
		h = 0.0;
	}

	return h;
}

// ----------------------------------------------------

