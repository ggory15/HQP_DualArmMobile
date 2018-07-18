#include "tasks/task-singularity.h"
namespace HQP
{
	namespace tasks
	{
		using namespace constraint;
		using namespace trajectories;

		TaskSingularityAvoidance::TaskSingularityAvoidance(const std::string & name, RobotModel & robot)
			: TaskBase(name, robot), m_constraint(name, 1, robot.nv()) {
			m_robot = robot;
			if (robot.type() == 0) {
				m_constraint.setLowerBound(-200.0 * VectorXd(robot.nv()).setOnes());
				m_constraint.setUpperBound(200.0 * VectorXd(robot.nv()).setOnes());
				m_Kp.setZero(robot.nv());
				m_Kd.setZero(robot.nv());
				VectorXd m = VectorXd::Ones(robot.nv());
				mask(m);
			}
			else if (robot.type() == 1) {
				m_constraint.setLowerBound(-2000.0 * VectorXd(1).setOnes());
				m_constraint.setUpperBound(2000.0 * VectorXd(1).setOnes());
				m_Kp.setZero(robot.nv());
				m_Kd.setZero(robot.nv());
				VectorXd m = VectorXd::Ones(robot.nv());
				mask(m);
			}
			m_buffer = 5.0 * M_PI / 180.0;
		}

		int TaskSingularityAvoidance::dim() const
		{
			return m_robot.nv();
		}

		const ConstraintBase & TaskSingularityAvoidance::compute(const double t, Cref_vectorXd q, Cref_vectorXd v) {
			MatrixXd A(1, m_robot.nv());
			A.setZero();
			VectorXd Jd = m_robot.getManipulabilityJacobian(0);
			for (int i = 0; i < dof / 2; i++) {
				A(0, i + 2) = Jd(i);
			}

			if (m_robot.getManipulability(0, q) < 0.05) {
				m_constraint.lowerBound() = 30.0 * (m_robot.getManipulability(0, q) - 0.005) * VectorXd(1).setOnes();
			}			
			else {
				A.setOnes();
			}
			m_constraint.setMatrix(A);

			//using namespace std;
			//cout << "A.row()" << A.rows() << A.cols() << endl;
			//cout << "mc" << m_constraint.upperBound().size() << endl;

			return m_constraint;
		}
		const ConstraintBase & TaskSingularityAvoidance::getConstraint() const {
			return m_constraint;
		}


		const  VectorXd & TaskSingularityAvoidance::mask() const {

		}
		void TaskSingularityAvoidance::mask(const VectorXd & mask) {

		}

		const VectorXd & TaskSingularityAvoidance::Kp() { return m_Kp; }

		const VectorXd & TaskSingularityAvoidance::Kd() { return m_Kd; }

		void TaskSingularityAvoidance::Kp(Cref_vectorXd Kp)
		{
			//assert(Kp.size() == m_robot.nv());
			m_Kp = Kp;
		}

		void TaskSingularityAvoidance::Kd(Cref_vectorXd Kd)
		{
			//assert(Kd.size() == m_robot.nv());
			m_Kd = Kd;
		}
	}
}
