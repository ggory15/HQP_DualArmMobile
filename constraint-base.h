#ifndef __HQP__Constraint__BASE__
#define __HQP__Constraint__BASE__

#include "fwd_constraints.h"

#include <string>

namespace HQP {
	namespace constraint {

		/*
		* @brief Abstract class representing a linear equality/inequality constraint.
		* Equality constraints are represented by a matrix A and a vector b: A*x = b
		* Inequality constraints are represented by a matrix A and two vectors
		* lb and ub: lb <= A*x <= ub
		* Bounds are represented by two vectors lb and ub: lb <= x <= ub
		*/

		class ConstraintBase {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			ConstraintBase(const std::string & name);

			ConstraintBase(const std::string & name,
				const unsigned int rows,
				const unsigned int cols);

			ConstraintBase(const std::string & name, Cref_matrixXd A);

			virtual const std::string & name() const;
			virtual unsigned int rows() const = 0;
			virtual unsigned int cols() const = 0;
			virtual void resize(const unsigned int r, const unsigned int c) = 0;

			virtual bool isEquality() const = 0;
			virtual bool isInequality() const = 0;
			virtual bool isBound() const = 0;

			virtual const MatrixXd & matrix() const;
			virtual const VectorXd & vector() const = 0;
			virtual const VectorXd & lowerBound() const = 0;
			virtual const VectorXd & upperBound() const = 0;

			virtual MatrixXd & matrix();
			virtual VectorXd & vector() = 0;
			virtual VectorXd & lowerBound() = 0;
			virtual VectorXd & upperBound() = 0;

			virtual bool setMatrix(Cref_matrixXd A);
			virtual bool setVector(Cref_vectorXd b) = 0;
			virtual bool setLowerBound(Cref_vectorXd lb) = 0;
			virtual bool setUpperBound(Cref_vectorXd ub) = 0;

			virtual bool checkConstraint(Cref_vectorXd x, double tol = 1e-6) const = 0;

		protected:
			std::string m_name;
			MatrixXd m_A;
		};
	}
}


#endif