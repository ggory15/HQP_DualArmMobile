#ifndef __HQP__Constraint__Bound__
#define __HQP__Constraint__Bound__

#include "constraint-base.h"

#include <string>

namespace HQP {
	namespace constraint {

		class ConstraintBound : public ConstraintBase {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			ConstraintBound(const std::string & name);

			ConstraintBound(const std::string & name, const unsigned int size);

			ConstraintBound(const std::string & name, Cref_vectorXd lb, Cref_vectorXd ub);

			unsigned int rows() const;
			unsigned int cols() const;
			void resize(const unsigned int r, const unsigned int c);

			bool isEquality() const;
			bool isInequality() const;
			bool isBound() const;

			const VectorXd & vector()     const;
			const VectorXd & lowerBound() const;
			const VectorXd & upperBound() const;

			VectorXd & vector();
			VectorXd & lowerBound();
			VectorXd & upperBound();

			bool setVector(Cref_vectorXd b);
			bool setLowerBound(Cref_vectorXd lb);
			bool setUpperBound(Cref_vectorXd ub);

			bool checkConstraint(Cref_vectorXd x, double tol = 1e-6) const;

		protected:
			VectorXd m_lb;
			VectorXd m_ub;
		};
	}
}


#endif // __HQP__Constraint__Bound__