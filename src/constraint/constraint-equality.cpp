#include "constraint/constraint-equality.h"

using namespace HQP::constraint;

ConstraintEquality::ConstraintEquality(const std::string & name) :
	ConstraintBase(name)
{}

ConstraintEquality::ConstraintEquality(const std::string & name,
	const unsigned int rows,
	const unsigned int cols) :
	ConstraintBase(name, rows, cols),
	m_b(VectorXd::Zero(rows))
{}

ConstraintEquality::ConstraintEquality(const std::string & name,
	Cref_matrixXd A,
	Cref_vectorXd b) :
	ConstraintBase(name, A),
	m_b(b)
{
	assert(A.rows() == b.rows());
}

unsigned int ConstraintEquality::rows() const
{
	assert(m_A.rows() == m_b.rows());
	return (unsigned int)m_A.rows();
}

unsigned int ConstraintEquality::cols() const
{
	return (unsigned int)m_A.cols();
}

void ConstraintEquality::resize(const unsigned int r, const unsigned int c)
{
	m_A.setZero(r, c);
	m_b.setZero(r);
}

bool ConstraintEquality::isEquality() const { return true; }
bool ConstraintEquality::isInequality() const { return false; }
bool ConstraintEquality::isBound() const { return false; }

const VectorXd & ConstraintEquality::vector()     const { return m_b; }
const VectorXd & ConstraintEquality::lowerBound() const { assert(false); return m_b; }
const VectorXd & ConstraintEquality::upperBound() const { assert(false); return m_b; }

VectorXd & ConstraintEquality::vector() { return m_b; }
VectorXd & ConstraintEquality::lowerBound() { assert(false); return m_b; }
VectorXd & ConstraintEquality::upperBound() { assert(false); return m_b; }

bool ConstraintEquality::setVector(Cref_vectorXd b) { m_b = b; return true; }
bool ConstraintEquality::setLowerBound(Cref_vectorXd) { assert(false); return false; }
bool ConstraintEquality::setUpperBound(Cref_vectorXd) { assert(false); return false; }

bool ConstraintEquality::checkConstraint(Cref_vectorXd x, double tol) const
{
	return (m_A*x - m_b).norm() < tol;
}

