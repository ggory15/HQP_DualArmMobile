#include "contact-3d.h"
#include "utils.h"


using namespace HQP::robot;
using namespace HQP::tasks;
using namespace HQP::constraint;
using namespace HQP::trajectories;

// simple force inequlaity task for point contact
// lb <= f <= ub

namespace HQP
{
	namespace contact
	{
		Contact3dPoint::Contact3dPoint(const std::string & name, RobotModel & robot,
			const int & frameid,
			Cref_vectorXd contactNormal,
			const double frictionCoefficient,
			const double minNormalForce,
			const double maxNormalForce) :
			ContactBase(name, robot),
			m_motionTask(name, robot, frameid),
			m_forceInequality(name, 5, 3), // 변수 수 체크,
			m_contactNormal(contactNormal),
			m_mu(frictionCoefficient),
			m_fMin(minNormalForce),
			m_fMax(maxNormalForce)	{
			updateForceInequalityConstraints();
		}

		void Contact3dPoint::updateForceInequalityConstraints()
		{
			Vector3d t1, t2;
			const int n_in = 4 + 1;
			const int n_var = 3 ;
			MatrixXd B = MatrixXd::Zero(n_in, n_var);
			VectorXd lb = -1e10*VectorXd::Ones(n_in);
			VectorXd ub = VectorXd::Zero(n_in);

			// check
			if (m_contactNormal == Vector3d::UnitX()) {
				t1 = m_contactNormal.cross(Vector3d::UnitY());
				if (t1.norm() < 1e-5)
					t1 = m_contactNormal.cross(Vector3d::UnitZ());
				t2 = m_contactNormal.cross(t1);
				t1.normalize();
				t2.normalize();
			}
			else if (m_contactNormal == Vector3d::UnitY()) {
				t1 = m_contactNormal.cross(Vector3d::UnitX());
				if (t1.norm() < 1e-5)
					t1 = m_contactNormal.cross(Vector3d::UnitZ());
				t2 = m_contactNormal.cross(t1);
				t1.normalize();
				t2.normalize();
			}
			else {
				t1 = m_contactNormal.cross(Vector3d::UnitX());
				if (t1.norm() < 1e-5)
					t1 = m_contactNormal.cross(Vector3d::UnitY());
				t2 = m_contactNormal.cross(t1);
				t1.normalize();
				t2.normalize();
			}

			B.block<1, 3>(0, 0) = (-t1 - m_mu*m_contactNormal).transpose();
			B.block<1, 3>(1, 0) = (t1 - m_mu*m_contactNormal).transpose();
			B.block<1, 3>(2, 0) = (-t2 - m_mu*m_contactNormal).transpose();
			B.block<1, 3>(3, 0) = (t2 - m_mu*m_contactNormal).transpose();

			B.block<1, 3>(n_in - 1, 0) = m_contactNormal.transpose();

			ub(n_in - 1) = m_fMax;
			lb(n_in - 1) = m_fMin;

			m_forceInequality.setMatrix(B);
			m_forceInequality.setLowerBound(lb);
			m_forceInequality.setUpperBound(ub);
		}

		double Contact3dPoint::getNormalForce(Cref_vectorXd f) const
		{
			assert(f.size() == n_force());
			double n = 0.0;
			for (int i = 0; i<4; i++)
				n += m_contactNormal.dot(f.segment<3>(i * 3));
			return n;
		}

		unsigned int Contact3dPoint::n_motion() const { return 0; }
		unsigned int Contact3dPoint::n_force() const { return 3; }

		const VectorXd & Contact3dPoint::Kp() const { return m_motionTask.Kp(); }
		const VectorXd & Contact3dPoint::Kd() const { return m_motionTask.Kd(); }
		void Contact3dPoint::Kp(Cref_vectorXd Kp) { m_motionTask.Kp(Kp); }
		void Contact3dPoint::Kd(Cref_vectorXd Kd) { m_motionTask.Kd(Kd); }


		bool Contact3dPoint::setContactNormal(Cref_vectorXd contactNormal)
		{
			assert(contactNormal.size() == 3);
			if (contactNormal.size() != 3)
				return false;
			m_contactNormal = contactNormal;
			updateForceInequalityConstraints();
			return true;
		}

		bool Contact3dPoint::setFrictionCoefficient(const double frictionCoefficient)
		{
			assert(frictionCoefficient>0.0);
			if (frictionCoefficient <= 0.0)
				return false;
			m_mu = frictionCoefficient;
			updateForceInequalityConstraints();
			return true;
		}

		bool Contact3dPoint::setMinNormalForce(const double minNormalForce)
		{
			assert(minNormalForce>0.0 && minNormalForce <= m_fMax);
			if (minNormalForce <= 0.0 || minNormalForce>m_fMax)
				return false;
			m_fMin = minNormalForce;
			VectorXd & lb = m_forceInequality.lowerBound();
			lb(lb.size() - 1) = m_fMin;
			return true;
		}

		bool Contact3dPoint::setMaxNormalForce(const double maxNormalForce)
		{
			assert(maxNormalForce >= m_fMin);
			if (maxNormalForce<m_fMin)
				return false;
			m_fMax = maxNormalForce;
			VectorXd & ub = m_forceInequality.upperBound();
			ub(ub.size() - 1) = m_fMax;
			return true;
		}

		void Contact3dPoint::setReference(const Transform3d & ref)
		{
			TrajectorySample s(12, 6);
			s.pos.head<3>() = ref.translation();
			typedef Eigen::Matrix<double, 9, 1> Vector9;
			s.pos.tail<9>() = Eigen::Map<const Vector9>(&ref.rotation()(0), 9);

			m_motionTask.setReference(s);
		}

		const ConstraintBase & Contact3dPoint::computeMotionTask(const double t,
			Cref_vectorXd q,
			Cref_vectorXd v)
		{
			return m_motionTask.compute(t, q, v);
		}

		const ConstraintInequality & Contact3dPoint::computeForceTask(const double,
			Cref_vectorXd,
			Cref_vectorXd)
		{
			return m_forceInequality;
		}

		double Contact3dPoint::getMinNormalForce() const { return m_fMin; }
		double Contact3dPoint::getMaxNormalForce() const { return m_fMax; }

		const TaskMotion & Contact3dPoint::getMotionTask() const { return m_motionTask; }

		const ConstraintBase & Contact3dPoint::getMotionConstraint() const { return m_motionTask.getConstraint(); }

		const ConstraintInequality & Contact3dPoint::getForceConstraint() const { return m_forceInequality; }

	}
}
