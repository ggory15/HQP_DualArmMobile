#ifndef __HQP__Contact_BASE__
#define __HQP__Contact_BASE__

#include "fwd.h"
#include "tasks/task-motion.h"

namespace HQP
{
	namespace contact
	{
		class ContactBase
		{
		public:

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			typedef constraint::ConstraintBase ConstraintBase;
			typedef constraint::ConstraintInequality ConstraintInequality;
			typedef constraint::ConstraintEquality ConstraintEquality;
			typedef tasks::TaskMotion TaskMotion;
			typedef robot::RobotModel RobotModel;

			ContactBase(const std::string & name, RobotModel & robot);

			const std::string & name() const;

			void name(const std::string & name);

			/// Return the number of motion constraints
			virtual unsigned int n_motion() const = 0;

			/// Return the number of force variables
			virtual unsigned int n_force() const = 0;

			virtual const ConstraintBase & computeMotionTask(const double t, Cref_vectorXd q, Cref_vectorXd v) = 0;

			virtual const ConstraintInequality & computeForceTask(const double t, Cref_vectorXd q, Cref_vectorXd v) = 0;

			//virtual const MatrixXd & getForceGeneratorMatrix() = 0;

			//virtual const ConstraintEquality & computeForceRegularizationTask(const double t, Cref_vectorXd q, Cref_vectorXd v) = 0;

			virtual const TaskMotion & getMotionTask() const = 0;
			virtual const ConstraintBase & getMotionConstraint() const = 0;
			virtual const ConstraintInequality & getForceConstraint() const = 0;
		//	virtual const ConstraintEquality & getForceRegularizationTask() const = 0;
		//	virtual double getForceRegularizationWeight() const = 0;

			virtual double getMinNormalForce() const = 0;
			virtual double getMaxNormalForce() const = 0;
			virtual bool setMinNormalForce(const double minNormalForce) = 0;
			virtual bool setMaxNormalForce(const double maxNormalForce) = 0;
			virtual double getNormalForce(Cref_vectorXd f) const = 0;

		protected:
			std::string m_name;
			RobotModel & m_robot;
		};
	}
}

#endif // ifndef __HQP__Contact_BASE__
