#include "constraint-bound.h"

using namespace HQP::constraint;

ConstraintBound::ConstraintBound(const std::string & name) :
	ConstraintBase(name)
{}

ConstraintBound::ConstraintBound(const std::string & name,
	const unsigned int size) :
	ConstraintBase(name, MatrixXd::Identity(size, size)),
	m_lb(VectorXd::Zero(size)),
	m_ub(VectorXd::Zero(size))
{}

ConstraintBound::ConstraintBound(const std::string & name,
	Cref_vectorXd lb,
	Cref_vectorXd ub) :
	ConstraintBase(name, MatrixXd::Identity(lb.size(), lb.size())),
	m_lb(lb),
	m_ub(ub)
{
	assert(lb.size() == ub.size());
}

unsigned int ConstraintBound::rows() const
{
	assert(m_lb.rows() == m_ub.rows());
	return (unsigned int)m_lb.rows();
}

unsigned int ConstraintBound::cols() const
{
	assert(m_lb.rows() == m_ub.rows());
	return (unsigned int)m_lb.rows();
}

void ConstraintBound::resize(const unsigned int r, const unsigned int c)
{
	assert(r == c);
	m_A.setIdentity(r, c);
	m_lb.setZero(r);
	m_ub.setZero(r);
}

bool ConstraintBound::isEquality() const { return false; }
bool ConstraintBound::isInequality() const { return false; }
bool ConstraintBound::isBound() const { return true; }

const VectorXd & ConstraintBound::vector()     const { assert(false); return m_lb; }
const VectorXd & ConstraintBound::lowerBound() const { return m_lb; }
const VectorXd & ConstraintBound::upperBound() const { return m_ub; }

VectorXd & ConstraintBound::vector() { assert(false); return m_lb; }
VectorXd & ConstraintBound::lowerBound() { return m_lb; }
VectorXd & ConstraintBound::upperBound() { return m_ub; }

bool ConstraintBound::setVector(Cref_vectorXd) { assert(false); return false; }
bool ConstraintBound::setLowerBound(Cref_vectorXd lb) { m_lb = lb; return true; }
bool ConstraintBound::setUpperBound(Cref_vectorXd ub) { m_ub = ub; return true; }

bool ConstraintBound::checkConstraint(Cref_vectorXd x, double tol) const
{
	return (x.array() <= m_ub.array() + tol).all() &&
		(x.array() >= m_lb.array() - tol).all();
}

