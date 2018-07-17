#include "solver-HQP-eiquadprog.h"
#include <iostream>
#include "eiquadprog_2011.h"

using namespace HQP::constraint;
using namespace HQP::solver;
using namespace Eigen;

using namespace std;
#define max_level 5
#define q_tol 0.001

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
		m_neq_t[i] = 0;
		m_nin_t[i] = 0;
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
		nin += m_nin[i];
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
			const unsigned int n = cl[0].second->cols();
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
			}
			if (m_neq[level_num] == neq && m_nin[level_num] == nin && m_nbound[level_num] == nbound)
				m_change[level_num] = false;
			else {
				m_change[level_num] = true; // constraint change!
				get_dim = true;
			}


			m_neq[level_num] = neq;
			m_nin[level_num] = nin;
			m_nbound[level_num] = nbound;
			m_n[level_num] = n;
			level_num++; // level_num 0에서 시작, 5에서 나옴
		}
	}
	
	if (get_dim == true) {
		for (int i = 0;i < max_level;i++) {
			for (int j = 0;j < i+1;j++)
			{
				m_neq_t[i] += m_neq[j];
				m_nin_t[i] += m_nin[j];
			}
		}
		get_dim = false;
	}
	//assert(level_num == 2, "task number must be more than 2");

	HQPData newHQPDdata;
	//newHQPDdata.resize(level_num - 1);

	for (int i = 0; i < max_level; i++)
		if (active_index(i) == 1) {
			newHQPDdata.push_back(problemData[i]);
		}

	//for (int i = 0; i < level_num - 1; i++)
	//	x_sol[i].setZero();

	unsigned int c_level = 1;
	int slack_total = 0;;

	for (int i = 0; i < level_num - 1; i++) { // level_num 5에서 나오니깐 i는 0부터 3까지( 0~ (level_num-1))

		if (m_change[c_level]) {
			resize_level(c_level);
		}
		resize_level(c_level);
		int i_eq = 0, i_in = 0, i_bound = 0, cc_level = 0;
		int c_eq = 0, c_in = 0;
		int slack = 0;
		if (c_level == level_num)
			break;

		while (cc_level <= c_level) {


			const ConstraintLevel & cl = newHQPDdata[cc_level];
			assert(cl.size() > 0);
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
				if (cc_level == c_level) {
					slack = constr->rows();
					slack_total += slack;
				}

				if (c_level >= 2 && cc_level == c_level) {

					for (int i = 1;i < c_level; i++)
					{	
						//cout << "input" << endl;
						m_ce0[c_level].segment(m_neq_t[c_level - i] - m_neq[c_level - i], m_neq[c_level - i]) += x_sol[c_level - i].tail(m_neq[c_level - i] + m_nin[c_level - i]).tail(m_neq[c_level - i]);
						m_ci0[c_level].segment(m_nin_t[c_level - i] - m_nin[c_level - i], m_nin[c_level - i]) -= x_sol[c_level - i].tail(m_neq[c_level - i] + m_nin[c_level - i]).head(m_nin[c_level - i]);
						m_ci0[c_level].segment(m_nin_t[c_level - i], m_nin[c_level - i]) += x_sol[c_level - i].tail(m_neq[c_level - i] + m_nin[c_level - i]).head(m_nin[c_level - i]);
					}

				} 

			}
			cc_level++;
		} // while 

		  // slack variable for lb and ub
		m_CI[c_level].block(m_CI[c_level].rows() - m_slack * 2 - m_nin[c_level] * 2, m_CI[c_level].cols() - m_slack, m_nin[c_level], m_nin[c_level]) = -1.0 * MatrixXd::Identity(m_nin[c_level], m_nin[c_level]);
		m_CI[c_level].block(m_CI[c_level].rows() - m_slack * 2 - m_nin[c_level], m_CI[c_level].cols() - m_slack, m_nin[c_level], m_nin[c_level]) = 1.0 * MatrixXd::Identity(m_nin[c_level], m_nin[c_level]);
		m_CI[c_level].block(m_CI[c_level].rows() - m_slack * 2, m_CI[c_level].cols() - m_slack, m_slack, m_slack) = -1.0 * MatrixXd::Identity(m_slack, m_slack);
		m_CI[c_level].bottomRightCorner(m_slack, m_slack) = 1.0 * MatrixXd::Identity(m_slack, m_slack);

		// m_slack > # of final slack varaibles
		m_ci0[c_level].tail(m_slack * 2) = 1000.0 * VectorXd(m_slack * 2).setOnes();
		m_ci0[c_level].tail(m_slack) = 1000.0 * VectorXd(m_slack).setOnes();


		// slack variable for Ce
		m_CE[c_level].bottomRightCorner(m_neq[c_level], m_neq[c_level]) = MatrixXd(m_neq[c_level], m_neq[c_level]).setIdentity();

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