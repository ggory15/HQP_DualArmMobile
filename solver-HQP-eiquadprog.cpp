#include "solver-HQP-eiquadprog.h"
#include <iostream>
#include "eiquadprog_2011.h"

using namespace HQP::constraint;
using namespace HQP::solver;
using namespace Eigen;

using namespace std;
#define max_level 5
#define q_tol 0.0001

SolverHQuadProg::SolverHQuadProg(const std::string & name) :
	SolverHQPBase(name),
	m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION)
{
	for (int i = 0; i < max_level; i++) {
		m_n[i] = 0;
		m_neq[i] = 0;
		m_nin[i] = 0;
		m_nbound[i] = 0;
		m_change[i] = false;
	}
}

void SolverHQuadProg::sendMsg(const std::string & s)
{
	std::cout << "[SolverHQuadProg." << m_name << "] " << s << std::endl;
}

void SolverHQuadProg::resize(unsigned int n, unsigned int neq, unsigned int nin)
{
}
void SolverHQuadProg::resize_level(unsigned int level) {
	assert(level > 0);

	unsigned int neq = 0, nin = 0, n_variable = m_n[0];

	for (unsigned int i = 0; i <= level; i++) {
		neq += m_neq[i];
		nin += m_nin[i] + m_nbound[i];
	}

	m_slack = m_neq[level] + m_nin[level];

	n_variable += m_neq[level] + m_nin[level];

	m_CE[level].resize(neq, n_variable);
	m_CE[level].setZero();
	m_ce0[level].resize(neq);
	m_ce0[level].setZero();
	
	m_CI[level].resize(nin * 2 + m_slack * 2, n_variable);
	m_CI[level].setZero();
	m_ci0[level].resize(nin * 2 + m_slack * 2);
	m_ci0[level].setZero();

	m_H[level].resize(n_variable, n_variable);
	m_H[level].setIdentity();
	m_H[level].topLeftCorner(m_n[0], m_n[0]) *= q_tol;// q_tol*MatrixXd(m_nbound[0], m_nbound[0]).setIdentity();

	m_g[level].resize(n_variable);
	m_g[level].setZero();
	
	x_sol[level].resize(n_variable);
	x_sol[level].setZero();

	
}
void SolverHQuadProg::resize(unsigned int n, unsigned int neq, unsigned int nin, unsigned int nbound) {

}

const HQPOutput & SolverHQuadProg::solve(const HQPData & problemData)
{
	VectorXi active_index(max_level);
	active_index.setZero();

	int level_num = 0;
	for (int i = 0; i < max_level; i++) {
		unsigned int neq = 0, nin = 0, nbound = 0;
		const ConstraintLevel & cl = problemData[i];
		if (cl.size() > 0) {
			active_index(i) = 1;
			const unsigned int n = cl[0].second->cols(); // dof
			for (ConstraintLevel::const_iterator it = cl.begin(); it != cl.end(); it++)
			{
				const ConstraintBase* constr = it->second;
				assert(n == constr->cols());
				if (constr->isEquality())
					neq += constr->rows();
				else if (constr->isInequality())
					nin += constr->rows();
				else if (constr->isBound())
					nbound = constr->lowerBound().size();
				if (i == 0) {
					assert(neq == 0 && nin == 0);	// The first Hierachical level must only joint limit!
					m_n[0] = nbound;
				}
			}
			if (m_neq[level_num] == neq && m_nin[level_num] == nin && m_nbound[level_num] == nbound)
				m_change[level_num] = false;
			else
				m_change[level_num] = true; // constraint change!

			
			m_neq[level_num] = neq;
			m_nin[level_num] = nin;
			m_nbound[level_num] = nbound;

			level_num++;
		}
	}

	for (int i = 0; i < max_level; i++)
		x_sol[i].setZero();

	unsigned int c_level = 1;
	for (int i = 0; i < max_level; i++) {
		if (m_change[c_level]) {
			resize_level(c_level);
		}
		resize_level(c_level);
		int i_eq = 0, i_in = 0, i_bound = 0, cc_level = 0;
		int c_eq = 0, c_in = 0;
		if (c_level == level_num)
			break;

		while (cc_level <= c_level) {
			if (active_index(cc_level) == 1) {
				//cout << "m_H" << m_H[c_level].transpose() << endl;
				//cout << "m_g" << m_g[c_level].transpose() << endl;
				//cout << "m_CI" << m_CI[c_level] << endl;
				//cout << "m_ci0" << m_ci0[c_level].transpose() << endl;
				//cout << "m_CE" << m_CE[c_level] << endl;
				//cout << "m_ce0" << m_ce0[c_level].transpose() << endl;
				//getchar();

				const ConstraintLevel & cl = problemData[cc_level];
				for (ConstraintLevel::const_iterator it = cl.begin(); it != cl.end(); it++)
				{
					c_eq = 0; c_in = 0;
					const ConstraintBase* constr = it->second;
					if (constr->isEquality())
					{
						m_CE[c_level].block(i_eq, 0, constr->rows(), constr->cols()) = constr->matrix();
						m_ce0[c_level].segment(i_eq, constr->rows()) = -constr->vector();
						i_eq += constr->rows();
						c_eq = constr->rows();
					}
					else if (constr->isInequality())
					{
						m_CI[c_level].block(i_in, 0, constr->rows(), constr->cols()) = -1.0 * constr->matrix();
						m_ci0[c_level].segment(i_in, constr->rows()) = constr->upperBound();
						i_in += constr->rows();
						m_CI[c_level].block(i_in, 0, constr->rows(), constr->cols()) = constr->matrix();
						m_ci0[c_level].segment(i_in, constr->rows()) = -1.0 * constr->lowerBound();
						i_in += constr->rows();
						c_in = constr->rows() * 2;
					}
					else if (constr->isBound())
					{
						m_CI[c_level].block(i_in, 0, constr->rows(), constr->rows()) = -MatrixXd::Identity(m_n[0], m_n[0]);
						m_ci0[c_level].segment(i_in, constr->rows()) = constr->upperBound();
						i_in += constr->rows();
						m_CI[c_level].block(i_in, 0, constr->rows(), constr->rows()) = MatrixXd::Identity(m_n[0], m_n[0]);
						m_ci0[c_level].segment(i_in, constr->rows()) = -1.0 * constr->lowerBound();
						i_in += constr->rows();
						c_in = constr->rows() * 2;
					}
					
					if (c_level >= 2 && cc_level == 2) {
						m_ce0[c_level].segment(0, 6) += x_sol[c_level - 1].segment(m_n[0], 6);
						// 이전 테스크의 eq 갯수를 받아와서 넣어줘야 함,..
						//m_ci0[c_level].segment(0, c_in) += x_sol[c_level - 1].segment(7, c_in);
					} //fix me
				}
				cc_level++;
			}
		} // while 

		
		  // slack variable for lb and ub
		m_CI[c_level].block(m_CI[c_level].rows() - m_slack * 2, m_CI[c_level].cols() - m_slack, m_slack, m_slack) = -1.0 * MatrixXd::Identity(m_slack, m_slack);
		m_CI[c_level].bottomRightCorner(m_slack, m_slack) = 1.0 * MatrixXd::Identity(m_slack, m_slack);
		m_ci0[c_level].tail(m_slack * 2) = 1000.0 * VectorXd(m_slack * 2).setOnes();
		m_ci0[c_level].tail(m_slack) = 1000.0 * VectorXd(m_slack).setOnes();

		
		// slack matrix for A
		m_CE[c_level].bottomRightCorner(m_slack, m_slack) = MatrixXd(m_slack, m_slack).setIdentity();

		//cout << "c" << c_eq + c_in << endl;
	

		//  min 0.5 * x G x + g0 x
		//  s.t.
		//  CE^T x + ce0 = 0
		//  CI^T x + ci0 >= 0
		m_objValue[c_level] = solve_quadprog(m_H[c_level], m_g[c_level], m_CE[c_level].transpose(), m_ce0[c_level], m_CI[c_level].transpose(), m_ci0[c_level], x_sol[c_level], m_activeSet[c_level], m_activeSetSize[c_level]);

		if (m_objValue[c_level] == std::numeric_limits<double>::infinity()) {
			m_output.status = HQP_STATUS_INFEASIBLE;
			cout << "m_H" << m_H[c_level].transpose() << endl;
			cout << "m_g" << m_g[c_level].transpose() << endl;
			cout << "m_CI" << m_CI[c_level] << endl;
			cout << "m_ci0" << m_ci0[c_level].transpose() << endl;
			cout << "m_CE" << m_CE[c_level] << endl;
			cout << "m_ce0" << m_ce0[c_level].transpose() << endl;
			getchar();
		}
		else
		{
			m_output.status = HQP_STATUS_OPTIMAL;
		}
		c_level++;
	}
	m_output.x = x_sol[c_level - 1];

	return m_output;
}

double SolverHQuadProg::getObjectiveValue()
{
	return 0; //  m_objValue[c_level];
}
double SolverHQuadProg::getObjectiveValue2(unsigned int level) {
	return m_objValue[level];
}
