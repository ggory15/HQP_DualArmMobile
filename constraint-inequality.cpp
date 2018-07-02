#include "constraint-inequality.h"

using namespace HQP::constraint;

ConstraintInequality::ConstraintInequality(const std::string & name) :
	ConstraintBase(name)
{}

ConstraintInequality::ConstraintInequality(const std::string & name,
	const unsigned int rows,
	const unsigned int cols) :
	ConstraintBase(name, rows, cols),
	m_lb(VectorXd::Zero(rows)),
	m_ub(VectorXd::Zero(rows))
{}

ConstraintInequality::ConstraintInequality(const std::string & name,
	Cref_matrixXd A,
	Cref_vectorXd lb,
	Cref_vectorXd ub) :
	ConstraintBase(name, A),
	m_lb(lb),
	m_ub(ub)
{
	assert(A.rows() == lb.rows());
	assert(A.rows() == ub.rows());
}

unsigned int ConstraintInequality::rows() const
{
	assert(m_A.rows() == m_lb.rows());
	assert(m_A.rows() == m_ub.rows());
	return (unsigned int)m_A.rows();
}

unsigned int ConstraintInequality::cols() const
{
	return (unsigned int)m_A.cols();
}

void ConstraintInequality::resize(const unsigned int r, const unsigned int c)
{
	m_A.setZero(r, c);
	m_lb.setZero(r);
	m_ub.setZero(r);
}

bool ConstraintInequality::isEquality() const { return false; }
bool ConstraintInequality::isInequality() const { return true; }
bool ConstraintInequality::isBound() const { return false; }

const VectorXd & ConstraintInequality::vector()     const { assert(false); return m_lb; }
const VectorXd & ConstraintInequality::lowerBound() const { return m_lb; }
const VectorXd & ConstraintInequality::upperBound() const { return m_ub; }

VectorXd & ConstraintInequality::vector() { assert(false); return m_lb; }
VectorXd & ConstraintInequality::lowerBound() { return m_lb; }
VectorXd & ConstraintInequality::upperBound() { return m_ub; }

bool ConstraintInequality::setVector(Cref_vectorXd) { assert(false); return false; }
bool ConstraintInequality::setLowerBound(Cref_vectorXd lb) { m_lb = lb; return true; }
bool ConstraintInequality::setUpperBound(Cref_vectorXd ub) { m_ub = ub; return true; }

bool ConstraintInequality::checkConstraint(Cref_vectorXd x, double tol) const
{
	return ((m_A*x).array() <= m_ub.array() + tol).all() &&
		((m_A*x).array() >= m_lb.array() - tol).all();
}

