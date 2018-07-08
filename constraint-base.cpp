#include "constraint-base.h"

using namespace HQP::constraint;

ConstraintBase::ConstraintBase(const std::string & name) :
	m_name(name) {}

ConstraintBase::ConstraintBase(const std::string & name,
	const unsigned int rows,
	const unsigned int cols) :
	m_name(name)
{
	m_A = MatrixXd::Zero(rows, cols);
}

ConstraintBase::ConstraintBase(const std::string & name,
	Cref_matrixXd A) :
	m_name(name),
	m_A(A)
{}

const std::string & ConstraintBase::name() const
{
	return m_name;
}

const MatrixXd & ConstraintBase::matrix() const
{
	return m_A;
}

MatrixXd & ConstraintBase::matrix()
{
	return m_A;
}

bool ConstraintBase::setMatrix(Cref_matrixXd A)
{
	m_A = A;
	return true;
}

