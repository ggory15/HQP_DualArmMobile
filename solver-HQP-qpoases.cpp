#include "solver-HQP-qpoases.h"
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
		m_n[i] = dof;
		m_neq[i] = 0;
		m_nin[i] = 0;
		m_nbound[i] = 0;
		m_change[i] = false;
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

	unsigned int neq = 0, nin = 0, nb = 0;

	for (unsigned int i = 0; i <= level; i++) {
		neq += m_neq[i];
		nin += m_nin[i];
		nb += m_nbound[i];
	}

	
	nb += m_neq[level] + m_nin[level];

	if (level == 2) {
		nb = 14;
		neq = 13;
		nin = 0;
	}


	m_A[level].resize(neq + nin, nb);
	m_A[level].setZero();
	m_Alb[level].resize(neq + nin);
	m_Alb[level].setZero();
	m_Aub[level].resize(neq + nin);
	m_Aub[level].setZero();

	m_H[level].resize(nb, nb);
	m_H[level].setIdentity();
	m_H[level].topLeftCorner(m_nbound[0], m_nbound[0]) *= q_tol;// q_tol*MatrixXd(m_nbound[0], m_nbound[0]).setIdentity();

	m_g[level].resize(nb);
	m_g[level].setZero();
	//fixme

	m_lb[level].resize(nb);
	m_ub[level].resize(nb);

	m_solver[level] = SQProblem(nb, neq+nin);
	m_solver[level].setOptions(m_options);
	m_init_succeeded = false;
	
	m_H_Row[level].resize(nb, nb);
	m_H_Row[level].setZero();
	m_A_Row[level].resize(neq + nin, nb);
	m_A_Row[level].setZero();
	x_sol[level].resize(nb);
	x_sol[level].setZero();
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
				if (i == 0)
					assert(neq == 0 && nin == 0);	// The first Hierachical level must only joint limit!
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
		int i_eq = 0, i_in = 0, i_bound = 0, cc_level = 0;
		int c_eq = 0, c_in = 0;
		if (c_level == level_num)
			break;

		while (cc_level <= c_level) {
			if (active_index(cc_level) == 1) {
				const ConstraintLevel & cl = problemData[cc_level];
				for (ConstraintLevel::const_iterator it = cl.begin(); it != cl.end(); it++)
				{
					c_eq = 0; c_in = 0;
					const ConstraintBase* constr = it->second;
					if (constr->isEquality())
					{
						m_A[c_level].block(i_eq + i_in, 0, constr->rows(), constr->cols()) = constr->matrix();
						m_Alb[c_level].segment(i_eq + i_in, constr->rows()) = constr->vector();
						m_Aub[c_level].segment(i_eq + i_in, constr->rows()) = constr->vector();

						i_eq += constr->rows();
						c_eq = constr->rows();
					}
					else if (constr->isInequality())
					{
						m_A[c_level].block(i_eq + i_in, 0, constr->rows(), constr->cols()) = constr->matrix();
						m_Alb[c_level].segment(i_eq + i_in, constr->rows()) = constr->lowerBound();
						m_Aub[c_level].segment(i_eq + i_in, constr->rows()) = constr->upperBound();
						i_in += constr->rows();
						c_in = constr->rows();
					}
					else if (constr->isBound())
					{
						m_lb[c_level].segment(i_bound, constr->rows()) = constr->lowerBound();
						m_ub[c_level].segment(i_bound, constr->rows()) = constr->upperBound();
						i_bound += constr->rows();
					}

					if (c_level == 2 && cc_level == 2) {
						m_Alb[c_level].segment(0, 6) -= x_sol[1].segment(7, 6);
						m_Aub[c_level].segment(0, 6) -= x_sol[1].segment(7, 6);

						//m_Alb[c_level].segment(i_eq + i_in - c_in, c_in) -= x_sol[c_level - 1].segment(i_in, c_in);
						//m_Aub[c_level].segment(i_eq + i_in - c_in, c_in) -= x_sol[c_level - 1].segment(i_in, c_in);
					}
				}				
				cc_level++;
			}
		} // while 

		// slack variable for lb and ub
		m_lb[c_level].tail(c_eq + c_in) = -100.0 * VectorXd(c_eq + c_in).setOnes();
		m_ub[c_level].tail(c_eq + c_in) = 100.0 * VectorXd(c_eq + c_in).setOnes();

		// slack matrix for A
		m_A[c_level].bottomRightCorner(c_eq + c_in, c_eq + c_in) = MatrixXd(c_eq + c_in, c_eq + c_in).setIdentity();

		//cout << "c" << c_eq + c_in << endl;
		////cout << "m_H" << m_H[c_level].transpose() << endl;
		////cout << "m_g" << m_g[c_level].transpose() << endl;
		////cout << "m_lb" << m_lb[c_level].transpose() << endl;
		////cout << "m_ub" << m_ub[c_level].transpose() << endl;
		////cout << "m_A" << m_A[c_level] << endl;
		////cout << "m_Aub" << m_Aub[c_level].transpose() << endl;

		m_H_Row[c_level] = m_H[c_level];
		m_A_Row[c_level] = m_A[c_level];


		int_t iter = 1000;
		if (!m_init_succeeded) {
			m_status = m_solver[c_level].init(m_H_Row[c_level].data(), m_g[c_level].data(), m_A_Row[c_level].data(), m_lb[c_level].data(), m_ub[c_level].data(), m_Alb[c_level].data(), m_Aub[c_level].data(), iter);
			if (m_status == SUCCESSFUL_RETURN) {
				m_init_succeeded = true;
			}
			else {
				cout << "m_H" << m_H[c_level].transpose() << endl;
				cout << "m_g" << m_g[c_level].transpose() << endl;
				cout << "m_lb" << m_lb[c_level].transpose() << endl;
				cout << "m_ub" << m_ub[c_level].transpose() << endl;
				cout << "m_A" << m_A[c_level] << endl;
				cout << "m_Aub" << m_Aub[c_level].transpose() << endl;
				cout << "m_Alb" << m_Alb[c_level].transpose() << endl;
			}
		}
		else
		{
			m_status = m_solver[c_level].hotstart(m_H_Row[c_level].data(), m_g[c_level].data(), m_A_Row[c_level].data(), m_lb[c_level].data(), m_ub[c_level].data(), m_Alb[c_level].data(), m_Aub[c_level].data(), iter);		
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
