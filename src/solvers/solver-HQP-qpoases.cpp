#include "solvers/solver-HQP-qpoases.h"
#include <iostream>

using namespace HQP::constraint;
using namespace HQP::solver;
using namespace Eigen;
using namespace qpOASES;
using namespace std;
#define max_level 5
#define q_tol 0.0001

Solver_HQP_qpoases::Solver_HQP_qpoases(const std::string & name): SolverHQPBase(name), m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION)
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
	
	  m_options.setToDefault();
	  m_options.initialStatusBounds = ST_INACTIVE;
	  m_options.printLevel          = PL_LOW; //PL_LOW
	  m_options.enableRegularisation = BT_TRUE;
	  m_options.enableEqualities = BT_TRUE;
}

void Solver_HQP_qpoases::sendMsg(const std::string & s)
{
  std::cout<<"[SolverHQP_QPOASES."<<m_name<<"] "<<s<<std::endl;
}

void Solver_HQP_qpoases::resize(unsigned int n, unsigned int neq, unsigned int nin, unsigned int nbound)
{
  //const bool resizeVar = n!=m_n;
  //const bool resizeEq = (resizeVar || neq!=m_neq );
  //const bool resizeIn = (resizeVar || nin!=m_nin );
  //const bool resizeBound = (resizeVar || nbound != m_nbound);

  //if(resizeEq)
  //{
  //  m_CE.resize(neq, n);
  //  m_ce0.resize(neq);
  //}
  //if(resizeIn)
  //{
  //  m_CI.resize(2*nin, n);
  //  m_ci0.resize(2*nin);
  //}
  //if(resizeVar)
  //{
  //  m_H.resize(n, n);
  //  m_g.resize(n);
  //  m_output.x.resize(n);
  //}
  //if (resizeBound)
  //{
	 // //assert(m_nbound == m_n);
	 // m_lb.resize(nbound);
	 // m_ub.resize(nbound);
  //}
  //
  //m_n = n;
  //m_neq = neq;
  //m_nin = nin;
  //m_nbound = nbound;

  //if (resizeEq || resizeIn || resizeVar){
  //  m_solver = SQProblem(m_n, m_neq + m_nin * 2);
  //  m_solver.setOptions(m_options);
  //  m_init_succeeded = false;
  //}

  //m_H_Row.resize(m_n, m_n);
  //m_H_Row.setZero();
  //m_A_Row.resize(m_neq + m_nin *2, m_n);
  //m_A_Row.setZero();
  //m_Alb.resize(m_neq + m_nin*2);
  //m_Alb.setZero();
  //m_Aub.resize(m_neq + m_nin*2);
  //m_Aub.setZero();
  //m_lb.setZero();
  //m_ub.setZero();
}   
void Solver_HQP_qpoases::resize_level(unsigned int level) {
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

	m_CI[level].resize(nin + m_slack, n_variable);
	m_CI[level].setZero();
	m_ci0[level].resize(nin + m_slack);
	m_ci0[level].setZero(); //upper bound
	m_ci1[level].resize(nin + m_slack);
	m_ci1[level].setZero(); // lower bound

	m_H_Row[level].resize(n_variable, n_variable);
	m_H_Row[level].setIdentity();
	m_H_Row[level].topLeftCorner(m_n[0], m_n[0]) *= q_tol;// q_tol*MatrixXd(m_nbound[0], m_nbound[0]).setIdentity();
	m_A_Row[level].resize(neq+nin + m_slack, n_variable);

	m_g[level].resize(n_variable);
	m_g[level].setZero();

	m_Alb[level].resize(neq + nin + m_slack);
	m_Alb[level].setZero();
	m_Aub[level].resize(neq + nin + m_slack);
	m_Aub[level].setZero();

	x_sol[level].resize(n_variable);
	x_sol[level].setZero();

	m_solver[level] = SQProblem(n_variable, neq + nin + m_slack);
   	m_solver[level].setOptions(m_options);
   	m_init_succeeded = false;  	
}

const HQPOutput & Solver_HQP_qpoases::solve(const HQPData & problemData) {

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
			level_num++; 
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

	unsigned int c_level = 1;
	int slack_total = 0;;

	for (int i = 0; i < level_num - 1; i++) { 

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
					m_ce0[c_level].segment(i_eq, constr->rows()) = constr->vector();
					i_eq += constr->rows();
					c_eq = constr->rows();
				}
				else if (constr->isInequality())
				{
					m_CI[c_level].block(i_in, 0, constr->rows(), constr->cols()) = constr->matrix();
					m_ci0[c_level].segment(i_in, constr->rows()) = constr->upperBound();
					m_ci1[c_level].segment(i_in, constr->rows()) = constr->lowerBound();
					i_in += constr->rows();
					c_in = constr->rows();
				}
				if (cc_level == c_level) {
					slack = constr->rows();
					slack_total += slack;
				}

				if (c_level >= 2 && cc_level == c_level) {

					for (int i = 1;i < c_level; i++)
					{	
						//cout << "input" << endl;
						m_ce0[c_level].segment(m_neq_t[c_level - i] - m_neq[c_level - i], m_neq[c_level - i]) -= x_sol[c_level - i].tail(m_neq[c_level - i] + m_nin[c_level - i]).tail(m_neq[c_level - i]);
						m_ci0[c_level].segment(m_nin_t[c_level - i] - m_nin[c_level - i], m_nin[c_level - i]) -= x_sol[c_level - i].tail(m_neq[c_level - i] + m_nin[c_level - i]).head(m_nin[c_level - i]);
						m_ci1[c_level].segment(m_nin_t[c_level - i] - m_nin[c_level - i], m_nin[c_level - i]) -= x_sol[c_level - i].tail(m_neq[c_level - i] + m_nin[c_level - i]).head(m_nin[c_level - i]);
					}
				} 

			}
			cc_level++;
		} // while 

		// slack variable for lb and ub
		m_CI[c_level].block(m_CI[c_level].rows() - m_slack - m_nin[c_level], m_CI[c_level].cols() - m_slack, m_nin[c_level], m_nin[c_level]) = MatrixXd::Identity(m_nin[c_level], m_nin[c_level]);
		m_CI[c_level].bottomRightCorner(m_slack, m_slack) = MatrixXd::Identity(m_slack, m_slack);

		// m_slack > # of final slack varaibles
		m_ci0[c_level].tail(m_slack) = 1000.0 * VectorXd(m_slack).setOnes();
		m_ci1[c_level].tail(m_slack) = -1000.0 * VectorXd(m_slack).setOnes();

		// slack variable for Ce
		m_CE[c_level].bottomRightCorner(m_neq[c_level], m_neq[c_level]) = MatrixXd(m_neq[c_level], m_neq[c_level]).setIdentity();

		//m_H_Row[c_level] = m_H[c_level];
		m_A_Row[c_level].topRows(m_CE[c_level].rows()) = m_CE[c_level];
		m_A_Row[c_level].bottomRows(m_CI[c_level].rows()) = m_CI[c_level];

		m_Aub[c_level].head(m_CE[c_level].rows()) = m_ce0[c_level];
		m_Alb[c_level].head(m_CE[c_level].rows()) = m_ce0[c_level];
		m_Aub[c_level].tail(m_CI[c_level].rows()) = m_ci0[c_level];
		m_Alb[c_level].tail(m_CI[c_level].rows()) = m_ci1[c_level];
		

		int_t iter = 1000;
		if (!m_init_succeeded) {
			m_status = m_solver[c_level].init(m_H_Row[c_level].data(), m_g[c_level].data(), m_A_Row[c_level].data(), 0, 0, m_Alb[c_level].data(), m_Aub[c_level].data(), iter);
			if (m_status == SUCCESSFUL_RETURN) {
				m_init_succeeded = true;
			}
			else {
				cout << "m_H" << m_H_Row[c_level].transpose() << endl;
				cout << "m_g" << m_g[c_level].transpose() << endl;
				cout << "m_A" << m_A_Row[c_level] << endl;
				cout << "m_Aub" << m_Aub[c_level].transpose() << endl;
				cout << "m_Alb" << m_Alb[c_level].transpose() << endl;
				getchar();
			}
		}
		else
		{
			m_status = m_solver[c_level].hotstart(m_H_Row[c_level].data(), m_g[c_level].data(), m_A_Row[c_level].data(), 0, 0, m_Alb[c_level].data(), m_Aub[c_level].data(), iter);		
			if (m_status != SUCCESSFUL_RETURN) {
				m_init_succeeded = false;
				cout << "m_H" << m_H[c_level].transpose() << endl;
				cout << "m_g" << m_g[c_level].transpose() << endl;
				cout << "m_lb" << m_lb[c_level].transpose() << endl;
				cout << "m_ub" << m_ub[c_level].transpose() << endl;
				cout << "m_A" << m_A[c_level] << endl;
				cout << "m_Aub" << m_Aub[c_level].transpose() << endl;
				cout << "m_Alb" << m_Alb[c_level].transpose() << endl;
			}
		}

		if (m_status == SUCCESSFUL_RETURN)
		{
			m_solver[c_level].getPrimalSolution(x_sol[c_level].data());
			m_solver[c_level].reset();
			m_init_succeeded = false;
		}

		if (getSimpleStatus(m_status) == -2)
		{
			getchar();
			cout << "ss" << m_H_Row[level_num-1] << endl;
			cout << "m_H" << m_H[c_level].transpose() << endl;
			cout << "m_g" << m_g[c_level].transpose() << endl;
			cout << "m_lb" << m_lb[c_level].transpose() << endl;
			cout << "m_ub" << m_ub[c_level].transpose() << endl;
			cout << "m_A" << m_A[c_level] << endl;
			cout << "m_Aub" << m_Aub[c_level].transpose() << endl;
			cout << "m_Alb" << m_Alb[c_level].transpose() << endl;
		}

		c_level++;
	}// for 
	
	int ss = getSimpleStatus(m_status);
	if (ss == 0)
		m_output.status = HQP_STATUS_OPTIMAL;
	else if (ss == 1)
		m_output.status = HQP_STATUS_MAX_ITER_REACHED;
	else if (ss == -1)
		m_output.status = HQP_STATUS_UNKNOWN;
	else if (ss == -2)
		m_output.status = HQP_STATUS_INFEASIBLE;
	else if (ss == -3)
		m_output.status = HQP_STATUS_UNBOUNDED;
  
	m_output.x = x_sol[c_level-1];
	
	return m_output;
}
double Solver_HQP_qpoases::getObjectiveValue() {
	return 0.0;
}
