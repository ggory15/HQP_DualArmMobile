#ifndef __HQP__Contact_3D__
#define __HQP__Contact_3D__

#include "contact-base.h"
#include "task-operational.h"
#include "constraint-inequality.h"
#include "constraint-equality.h"

namespace HQP
{
	namespace contact
	{
		class Contact3dPoint : public ContactBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			typedef constraint::ConstraintInequality ConstraintInequality;
			typedef constraint::ConstraintEquality ConstraintEquality;
			typedef tasks::TaskMotion TaskMotion;
			typedef tasks::TaskOperationalSpace TaskOperationalSpace;

			Contact3dPoint(const std::string & name, RobotModel & robot,
				const int & frameid,
				Cref_vectorXd contactNormal, // object normal
				const double frictionCoefficient, 
				const double minNormalForce,
				const double maxNormalForce
			);

			/// Return the number of motion constraints
			virtual unsigned int n_motion() const;

			/// Return the number of force variables
			virtual unsigned int n_force() const;

			virtual const ConstraintBase & computeMotionTask(const double t,
				Cref_vectorXd q,
				Cref_vectorXd v);

			virtual const ConstraintInequality & computeForceTask(const double t,
				Cref_vectorXd q,
				Cref_vectorXd v);

			const TaskMotion & getMotionTask() const;
			const ConstraintBase & getMotionConstraint() const;
			const ConstraintInequality & getForceConstraint() const;

			double getNormalForce(Cref_vectorXd f) const;
			double getMinNormalForce() const;
			double getMaxNormalForce() const;

			const VectorXd & Kp() const;
			const VectorXd & Kd() const;
			void Kp(Cref_vectorXd Kp);
			void Kd(Cref_vectorXd Kp);

			bool setContactNormal(Cref_vectorXd contactNormal);

			bool setFrictionCoefficient(const double frictionCoefficient);
			bool setMinNormalForce(const double minNormalForce);
			bool setMaxNormalForce(const double maxNormalForce);
			void setReference(const Transform3d & ref);
			
		protected:

			void updateForceInequalityConstraints();

			TaskOperationalSpace m_motionTask;
			ConstraintInequality m_forceInequality;

			Vector3d m_contactNormal;
			Vector6d m_fRef;
			Vector6d m_weightForceRegTask;
			double m_mu;
			double m_fMin;
			double m_fMax;
			MatrixXd m_forceGenMat;
		};
	}
}

#endif // ifndef __HQP__Contact_3D__
