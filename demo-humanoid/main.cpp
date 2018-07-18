#include <iostream>
#include "vrep_bridge/vrep_bridge.h"

//for controller 
#include "controller/Inverse-dynamics.h"

// for tasks
#include "tasks/task-com.h"
#include "tasks/task-operational.h"
#include "tasks/task-joint-posture.h"
#include "tasks/task-joint-bounds.h"
#include "tasks/task-mobile.h"
#include "tasks/task-singularity.h"

// for trajectories 
#include "trajectories/trajectory-operationalspace.h"
#include "trajectories/trajectory-jointspace.h"

// for solver
#include "solvers/solver-HQP-factory.hxx"
#include "solvers/solver-utils.h"
#include "solvers/solver-HQP-eiquadprog.h"
#include "solvers/solver-HQP-qpoases.h"
//#include <tsid/math/utils.hpp>

// for contact point

#include "contacts/contact-3d.h"
#include "utils/container.h"

#include <string>
#include <vector>
//#include <conio.h> // for keyboard hit

// for vrep keyboard event
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

HQP::robot::RobotModel * robot_;
HQP::InverseDynamics * invdyn_;
HQP::tasks::TaskJointPosture * postureTask;
HQP::tasks::TaskOperationalSpace * moveTask, * move2Task;
HQP::tasks::TaskMobile * mobileTask;
HQP::tasks::TaskJointLimit * jointLimitTask;
HQP::contact::Contact3dPoint * contactTask;
HQP::tasks::TaskSingularityAvoidance * singularTask;


VectorXd q(dof);
VectorXd qdot(dof);
VectorXd qdes(dof);
VectorXd q_lb(dof + 2); // mobile 2 + robot 7
VectorXd q_ub(dof + 2); // mobile 2 + robot 7
double vrep_time = 0.0;
double Hz = 1000.0;
int na;
int nv;
int nq;

using namespace HQP;
using namespace std;

int main()
{
   robot_ = new HQP::robot::RobotModel(1);
   na = robot_->na();	
   nv = robot_->nv();

   invdyn_ = new HQP::InverseDynamics(*robot_);
   q.setZero();
   qdot.setZero();	

   solver::SolverHQPBase * solver = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_QPOASES, "solver-eiquadprog");
	

	// Level 0 : Joint Velocity Limit for Mobile + Manipulator 
	q_lb = -100.0 / 180.0 * M_PI * VectorXd(dof + 2).setOnes();
	q_ub = -1.0*q_lb;
	//q_ub(5) = 45.0 * M_PI / 180.0;
	q_lb.head(2) = -30.0 * VectorXd(2).setOnes();
	q_ub.head(2) = 30.0 * VectorXd(2).setOnes();

	double kp_jointlimit = 100.0, w_jointlimit = 1.00;
	
	singularTask = new tasks::TaskSingularityAvoidance("singular", *robot_);
	singularTask->Kp(kp_jointlimit * VectorXd::Ones(1));
	singularTask->Kd(2.0*singularTask->Kp().cwiseSqrt());
	
	//invdyn_->addSingularityTask(*singularTask, 1.0, 1, 0.0);
	
	jointLimitTask = new tasks::TaskJointLimit("joint_limit_task", *robot_);
	jointLimitTask->Kp(kp_jointlimit*VectorXd::Ones(robot_->nv()));
	jointLimitTask->Kd(4.0*jointLimitTask->Kp().cwiseSqrt());
	jointLimitTask->setJointLimit(q_lb, q_ub);

	invdyn_->addJointLimitTask(*jointLimitTask, 0.0,0, 0.0);


	qdes = 20.0 / 180.0 * 3.14*VectorXd(dof).setOnes();
	qdes.setZero();
	qdes(3) = -50.0 * M_PI/180.0;
	qdes(10) = -50.0 * M_PI / 180.0;
	
	postureTask = new tasks::TaskJointPosture("joint_control_task", *robot_);
	double kp_posture = 300.0, w_posture = 1.00;
	postureTask->Kp(kp_posture*VectorXd::Ones(robot_->nv()-2));
	postureTask->Kd(2.0*postureTask->Kp().cwiseSqrt());
	invdyn_->addJointPostureTask(*postureTask, 1.0, 2, 0.0);

	moveTask = new tasks::TaskOperationalSpace("end_effector_task", *robot_, 7);
	double kp_move = 100.0, w_move = 1.0;
	VectorXd a = VectorXd::Ones(6);
	a.tail(3) *= 10.0;
	moveTask->Kp(kp_move*a);
	moveTask->Kd(2.0*moveTask->Kp().cwiseSqrt());
	moveTask->setSingular(false);

	move2Task = new tasks::TaskOperationalSpace("end_effector_task2", *robot_, 7);
	move2Task->Kp(kp_move*a);
	move2Task->Kd(2.0*move2Task->Kp().cwiseSqrt());
	move2Task->setSingular(true);

	mobileTask = new tasks::TaskMobile("mobile_task", *robot_);
	double kp_mobile = 100.0, w_mobile = 1.0; // 400 -> 9 deg 
	mobileTask->Kp(kp_mobile*VectorXd::Ones(6));
	mobileTask->Kd(2.0*mobileTask->Kp().cwiseSqrt());
	mobileTask->setOnlyOriCTRL(true);



	trajectories::TrajectoryBase *trajPosture = new trajectories::TrajectoryJointConstant("joint_traj", qdes);
	trajectories::TrajectorySample samplePosture(robot_->nv()-2);
	//
	Transform3d T_endeffector;
	Transform3d init_T, goal_T;
	double duration, stime;
	trajectories::TrajectorySample s(12, 6);
	trajectories::TrajectoryOperationConstant *trajEE = new trajectories::TrajectoryOperationConstant("operational_traj", T_endeffector);
	//

	Transform3d T_mobile;
	trajectories::TrajectorySample s_mobile(12, 6);
	trajectories::TrajectoryOperationConstant *trajmobile = new trajectories::TrajectoryOperationConstant("mobile_traj", T_mobile);

	solver->resize(invdyn_->nVar(), invdyn_->nEq(), invdyn_->nIn(), invdyn_->nBound());
	// for v-rep
	VRepBridge vb;
	vb.isSimulationRun = false;
	vb.exitFlag = false;

	bool flag = false;
	while (vb.simConnectionCheck() && !vb.exitFlag)
	{
		if (_kbhit()) {
			int key;
			key = getchar();
			switch (key)
			{
			case '\t':
				if (vb.isSimulationRun) {
					cout << "Simulation Pause" << endl;
					vb.isSimulationRun = false;
				}
				else {
					cout << "Simulation Run" << endl;
					vb._cntt = 0;
					vb.isSimulationRun = true;
				}
				break;

			case 'q':
				vb.isSimulationRun = false;
				vb.exitFlag = true;
				simxEndDialog(vb.getClientID(), vb.dialog_handle, simx_opmode_oneshot);
				break;

			default:
				break;
			}
		}
		if (vb.isSimulationRun)
		{
			vb.read();
			
			if (vb._cntt > 0)
			{
				vrep_time = vb._cntt / Hz;

				VectorXd q_current(robot_->na() + 5), qdot_current(robot_->na() + 5);			
				
				q_current.setZero(); 
				q_current.head<2>() = vb.H_transform_.translation().head(2);
				q_current(2) = vb.euler_(2);
				q_current(3) = 0.0;
				q_current(4) = 0.0;			
				q_current.tail<dof>() = vb.current_q_;

				qdot_current.setZero();
				qdot_current.head<2>() = vb.H_vel_.linear().head(2);
				qdot_current(2) = vb.H_vel_.angular()(2);
				qdot_current(3) = vb.current_base_vel_(0);
				qdot_current(4) = vb.current_base_vel_(1);
				qdot_current.tail(dof) = vb.current_qdot_;
				
				robot_->getUpdateKinematics(q_current, qdot_current);

				if (vrep_time == 1.0 / Hz) {
					init_T = robot_->getTransformation(7);
					goal_T = init_T;
					goal_T.translation()(0) += 0.33;
					T_endeffector = robot_->getTransformation(7);
					T_endeffector.translation()(0) += 0.33;
					//T_endeffector.translation()(1) -= 0.13;
		
					trajEE->setReference(T_endeffector);		
					invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
					invdyn_->addOperationalTask(*move2Task, w_move, 1, 0.0);
					
					T_mobile = robot_->getMobileTransformation();
					trajmobile->setReference(T_mobile);
					s_mobile = trajmobile->computeNext();
					invdyn_->addMotionTask(*mobileTask, w_mobile, 3, 0.0);
					mobileTask->setReference(s_mobile);

				}

				s = trajEE->computeNext();
				moveTask->setReference(s);
				move2Task->setReference(s);

				MatrixXd J_EE_left(6, 7); 
				J_EE_left = robot_->getJacobian(7).block(0, 2, 6, 7);

				s = trajEE->computeNext();
				moveTask->setReference(s);
				move2Task->setReference(s);
				s_mobile = trajmobile->computeNext();
				mobileTask->setReference(s_mobile);

				samplePosture = trajPosture->computeNext();
				postureTask->setReference(samplePosture);
				const solver::HQPData & HQPData = invdyn_->computeProblemData(vrep_time, q_current, qdot_current);

				if (vrep_time == 1.0/Hz)
					cout << solver::HQPDataToString(HQPData, true) << endl;				
								
				const solver::HQPOutput & sol = solver->solve(HQPData);
				const VectorXd & tau = invdyn_->getActuatorForces(sol);
				VectorXd dv = invdyn_->getAccelerations(sol);


				vb.desired_base_vel_(0) = dv(0);
				vb.desired_base_vel_(1) = dv(1);
				vb.desired_base_vel_(2) = dv(1);
				vb.desired_base_vel_(3) = dv(0);

				vb.desired_torque_ = tau;
				vb.write();

			}		
			vb.simLoop();
			vb._cntt++;

		}
	}

	return 0;
}
