#ifndef __inv_dyn_H
#define __inv_dyn_H

#include "fwd.h"
// for robot model
#include "robot/robot_model.h"
// for constraint
#include "constraint/constraint-equality.h"
#include "constraint/constraint-inequality.h"
#include "constraint/constraint-bound.h"
// for tasks
#include "tasks/task-com.h"
#include "tasks/task-joint-posture.h"
#include "tasks/task-operational.h"
#include "tasks/task-joint-bounds.h"
#include "tasks/task-mobile.h"
#include "tasks/task-singularity.h"
// for solvers
#include "solvers/solver-HQP-qpoases.h"
// for contact
#include "contacts/contact-3d.h"
#include "contacts/contact-base.h"

#include <string>
#include <vector>

namespace HQP {

	class TaskLevel
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tasks::TaskBase & task;
		constraint::ConstraintBase * constraint;

		unsigned int priority;

		 TaskLevel(tasks::TaskBase & task, unsigned int priority);
	};
	class ContactLevel
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		contact::ContactBase & contact;
		constraint::ConstraintBase * motionConstraint;
		constraint::ConstraintInequality * forceConstraint;
		constraint::ConstraintEquality * forceRegTask;
		unsigned int index; /// index of 1st element of associated force variable in the force vector

		ContactLevel(contact::ContactBase & contact);
	};

	class InverseDynamics {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		typedef robot::RobotModel RobotModel;
		typedef tasks::TaskBase TaskBase;
		typedef tasks::TaskMotion TaskMotion;
		// typedef tasks::TaskCom TaskCom;
		typedef tasks::TaskJointPosture TaskJointPosture;
		typedef tasks::TaskOperationalSpace TaskSE3Equality;
		typedef tasks::TaskJointLimit TaskJointLimit;
		typedef tasks::TaskSingularityAvoidance TaskSingularityAvoidance;
		//typedef tasks::TaskContactForce TaskContactForce;
		//typedef tasks::TaskActuation TaskActuation;
		typedef solver::HQPOutput HQPOutput;
		typedef solver::HQPData HQPData;
		typedef contact::ContactBase ContactBase;

		InverseDynamics(RobotModel & robot, bool verbose = false);
		~InverseDynamics() {}

		unsigned int nVar() const;
		unsigned int nEq() const;
		unsigned int nIn() const;
		unsigned int nBound() const;

		bool addMotionTask(TaskMotion & task, double weight, unsigned int priorityLevel, double transition_duration = 0.0);
		bool addJointPostureTask(TaskJointPosture & task, double weight, unsigned int priorityLevel, double transition_duration = 0.0);
		bool addJointLimitTask(TaskJointLimit & task, double weight, unsigned int priorityLevel, double transition_duration = 0.0);
		bool addSingularityTask(TaskSingularityAvoidance & task, double weight, unsigned int priorityLevel, double transition_duration = 0.0);
		bool addOperationalTask(TaskSE3Equality & task, double weight, unsigned int priorityLevel, double transition_duration = 0.0);
		bool addRigidContact(ContactBase & contact, unsigned int priorityLevel);


		bool updateTaskWeight(const std::string & task_name, double weight);
		bool removeTask(const std::string & taskName, double transition_duration = 0.0);
		const HQPData & computeProblemData(double time,  VectorXd q, VectorXd v);

		const VectorXd & getActuatorForces(const HQPOutput & sol);
		const VectorXd & getContactForces(const HQPOutput & sol);
		const VectorXd & getAccelerations(const HQPOutput & sol);
		const VectorXd & getSlack(const HQPOutput & sol);
		HQPData & getHQPData() { return m_hqpData; };

		void addTask(TaskLevel* task, double weight, unsigned int priorityLevel);
		void resizeHqpData();
		bool removeFromHqpData(const std::string & name);
		bool decodeSolution(const HQPOutput & sol);
		bool checkTask(const std::string & name);


	public:
		bool m_verbose;
		RobotModel m_robot;
		HQPData m_hqpData;
		solver::Solver_HQP_qpoases m_solver;
		HQPOutput m_sol;
		std::vector<TaskLevel*>     m_taskMotions;
		std::vector<ContactLevel*>   m_contacts;

		double m_t;         /// time
		unsigned int m_k;   /// number of contact-force variables
		unsigned int m_v;   /// number of acceleration variables
		unsigned int m_eq;  /// number of equality constraints
		unsigned int m_in;  /// number of inequality constraints
		unsigned int m_bound;
		MatrixXd m_Jc;        /// contact force Jacobian
		constraint::ConstraintEquality m_baseDynamics;

		bool m_solutionDecoded;
		VectorXd m_dv;
		VectorXd m_f;
		VectorXd m_tau;
		VectorXd m_slack;
		
		//std::vector<ContactTransitionInfo*> m_contactTransitions;
	};

}

#endif