#include "solver-HQP-qpoases.h"
//#include "utils.h"
#include <iostream>
//#include "tsid/solvers/eiquadprog_2011.hpp"


using namespace HQP::constraint;
using namespace HQP::solver;
using namespace Eigen;
using namespace qpOASES;
using namespace std;

Solver_HQP_qpoases::Solver_HQP_qpoases(const std::string & name): SolverHQPBase(name), m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION)
{
  m_n = 0;
  m_neq = 0;
  m_nin = 0;

  m_options.setToDefault();
  m_options.printLevel          = PL_LOW; //PL_LOW
  m_options.enableRegularisation = BT_TRUE;
  m_options.enableEqualities = BT_TRUE;
}

void Solver_HQP_qpoases::sendMsg(const std::string & s)
{
  std::cout<<"[SolverHQP_QPOASES."<<m_name<<"] "<<s<<std::endl;
}

void Solver_HQP_qpoases::resize(unsigned int n, unsigned int neq, unsigned int nin)
{
  const bool resizeVar = n!=m_n;
  const bool resizeEq = (resizeVar || neq!=m_neq );
  const bool resizeIn = (resizeVar || nin!=m_nin );

  if(resizeEq)
  {
    m_CE.resize(neq, n);
    m_ce0.resize(neq);
  }
  if(resizeIn)
  {
    m_CI.resize(2*nin, n);
    m_ci0.resize(2*nin);
  }
  if(resizeVar)
  {
    m_H.resize(n, n);
    m_g.resize(n);
    m_output.x.resize(n);
  }

  m_n = n;
  m_neq = neq;
  m_nin = nin;

  if (resizeEq || resizeIn || resizeVar){
    m_solver = SQProblem(m_n, m_neq + m_nin * 2);
    m_solver.setOptions(m_options);
    m_init_succeeded = false;
  }

  m_H_Row.resize(m_n, m_n);
  m_H_Row.setZero();
  m_A_Row.resize(m_neq + m_nin *2, m_n);
  m_A_Row.setZero();
  m_Alb.resize(m_neq + m_nin*2);
  m_Alb.setZero();
  m_Aub.resize(m_neq + m_nin*2);
  m_Aub.setZero();
}   
const HQPOutput & Solver_HQP_qpoases::solve(const HQPData & problemData){
	if(problemData.size()>2)
	{
		assert(false && "Solver not implemented for more than 2 hierarchical levels.");
	}

	// Compute the constraint matrix sizes
	unsigned int neq = 0, nin = 0;
	const ConstraintLevel & cl0 = problemData[0];
	
	if(cl0.size()>0)
	{
		const unsigned int n = cl0[0].second->cols();
		for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
		{
			const ConstraintBase* constr = it->second;
			assert(n==constr->cols());
			if(constr->isEquality())
			neq += constr->rows();
			else
			nin += constr->rows();
		}

		resize(n, neq, nin);
		int i_eq=0, i_in=0;
		for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
		{
			const ConstraintBase* constr = it->second;
			if(constr->isEquality())
			{
				m_CE.middleRows(i_eq, constr->rows()) = constr->matrix();
				m_ce0.segment(i_eq, constr->rows())   = -constr->vector();
				i_eq += constr->rows();
			}
			else if(constr->isInequality())
			{
				m_CI.middleRows(i_in, constr->rows()) = constr->matrix();
				m_ci0.segment(i_in, constr->rows())   = -constr->lowerBound();
				i_in += constr->rows();
				m_CI.middleRows(i_in, constr->rows()) = -constr->matrix();
				m_ci0.segment(i_in, constr->rows())   = constr->upperBound();
				i_in += constr->rows();
			}
			else if(constr->isBound())
			{
				m_CI.middleRows(i_in, constr->rows()).setIdentity();
				m_ci0.segment(i_in, constr->rows())   = -constr->lowerBound();
				i_in += constr->rows();
				m_CI.middleRows(i_in, constr->rows()) = -MatrixXd::Identity(m_n, m_n);
				m_ci0.segment(i_in, constr->rows())   = constr->upperBound();
				i_in += constr->rows();
			} // fix me
		}
	}
	else
		resize(m_n, neq, nin);


	if(problemData.size()>1)
	{
		const ConstraintLevel & cl1 = problemData[1];
		m_H.setZero();
		m_g.setZero();
		for(ConstraintLevel::const_iterator it=cl1.begin(); it!=cl1.end(); it++)
		{
			const double & w = it->first;
			const ConstraintBase* constr = it->second;
			if(!constr->isEquality())
			assert(false && "Inequalities in the cost function are not implemented yet");

			m_H += w*constr->matrix().transpose()*constr->matrix();
			m_g -= w*(constr->matrix().transpose()*constr->vector());
		}
		m_H.diagonal() += m_hessian_regularization*VectorXd::Ones(m_n);
	}

  
	m_H_Row = m_H; 
	m_A_Row.topLeftCorner(m_neq, m_n) = m_CE;
	m_A_Row.bottomLeftCorner(m_nin*2, m_n) = m_CI;
    
	m_Alb.head(m_neq) = -1.0*m_ce0;
	m_Alb.tail(m_nin*2) = -1.0*m_ci0;  

	m_Aub.head(m_neq) = -1.0*m_ce0;
	m_Aub.tail(m_nin*2) = 1000.0 * VectorXd(m_nin*2).setOnes();

	int_t iter = 1000;
	VectorXd x_sol(m_n);
	if (!m_init_succeeded){
		m_status = m_solver.init(m_H_Row.data(), m_g.data(), m_A_Row.data(), 0, 0, m_Alb.data(), m_Aub.data(), iter);
		if(m_status==SUCCESSFUL_RETURN){
			m_init_succeeded = true;
		}
	}
	else
	{
		m_status = m_solver.hotstart(m_H_Row.data(), m_g.data(), m_A_Row.data(), 0, 0, m_Alb.data(), m_Aub.data(), iter);
		if(m_status!=SUCCESSFUL_RETURN)
			m_init_succeeded = false;
	}

	if(m_status==SUCCESSFUL_RETURN)
	{
		m_solver.getPrimalSolution(x_sol.data());
	}

	m_objValue = m_solver.getObjVal();

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
  
	m_output.x = x_sol;
	
	return m_output;
}

double Solver_HQP_qpoases::getObjectiveValue()
{
  return m_objValue;
}