#include "Inverse-dynamics.h"


using namespace HQP;
using namespace constraint;
using namespace tasks;
using namespace solver;

TaskLevel::TaskLevel(tasks::TaskBase & task,
	unsigned int priority) :
	task(task),
	priority(priority)
{}

InverseDynamics::InverseDynamics(RobotModel & robot, bool verbose)
	: m_robot(robot), m_verbose(verbose), m_baseDynamics("base-dynamics", 6, robot.nv()), m_solutionDecoded(false), m_solver("name") {
	m_t = 0.0;
	m_v = robot.nv();
	m_k = 0;

	if (m_robot.type() == 0)
		m_eq = 0;
	else if (m_robot.type() == 1)
		m_eq = 3;
	else if (m_robot.type() == 2)
		m_eq = 6;

	m_in = 0;
	m_bound = 0;
	m_hqpData.resize(5);
	//m_Jc.setZero(m_k, m_v);
	//m_hqpData[0].push_back(make_pair<double, ConstraintBase*>(1.0, &m_baseDynamics));
}	
unsigned int InverseDynamics::nVar() const
{
	return m_v + m_k;
}

unsigned int InverseDynamics::nEq() const
{
	return m_eq;
}

unsigned int InverseDynamics::nIn() const
{
	return m_in;
}

unsigned int InverseDynamics::nBound() const
{
	return m_bound;
}
void InverseDynamics::resizeHqpData()
{
	//m_Jc.setZero(m_k, m_v);
	m_baseDynamics.resize(6, m_v + m_k);
	for (HQPData::iterator it = m_hqpData.begin(); it != m_hqpData.end(); it++)
	{
		for (ConstraintLevel::iterator itt = it->begin(); itt != it->end(); itt++)
		{
			itt->second->resize(itt->second->rows(), m_v + m_k);
		}
	}
}
void InverseDynamics::addTask(TaskLevel* tl, double weight, unsigned int priorityLevel)
{
	if (priorityLevel > m_hqpData.size())
		m_hqpData.resize(priorityLevel);
	const ConstraintBase & c = tl->task.getConstraint();
	if (c.isEquality())
	{
		tl->constraint = new ConstraintEquality(c.name(), c.rows(), m_v + m_k);
		if (priorityLevel == 0)
			m_eq += c.rows();
	}
	else if (c.isInequality())
	{
		tl->constraint = new ConstraintInequality(c.name(), c.rows(), m_v + m_k);
		if (priorityLevel == 0)
			m_in += c.rows();
	}
	else
	{
		tl->constraint = new ConstraintBound(c.name(), m_v + m_k);
		m_bound += c.lowerBound().size();
	}
	m_hqpData[priorityLevel].push_back(make_pair<double, ConstraintBase*>(weight, tl->constraint));
}

bool InverseDynamics::addMotionTask(TaskMotion & task, double weight, unsigned int priorityLevel, double transition_duration)
{
	assert(weight >= 0.0);
	assert(transition_duration >= 0.0);

	// This part is not used frequently so we can do some tests.
	if (weight >= 0.0)
		std::cerr << __FILE__ << " " << __LINE__ << " "
		<< "weight should be positive" << std::endl;

	// This part is not used frequently so we can do some tests.
	if (transition_duration >= 0.0) {
		std::cerr << "transition_duration should be positive" << std::endl;
	}

	TaskLevel *tl = new TaskLevel(task, priorityLevel);
	m_taskMotions.push_back(tl);
	addTask(tl, weight, priorityLevel);

	return true;
}

bool InverseDynamics::addJointPostureTask(TaskJointPosture & task, 	double weight, 	unsigned int priorityLevel, double transition_duration)
{
	assert(weight >= 0.0);
	assert(transition_duration >= 0.0);

	// This part is not used frequently so we can do some tests.
	if (weight >= 0.0)
		std::cerr << __FILE__ << " " << __LINE__ << " "
		<< "weight should be positive" << std::endl;

	// This part is not used frequently so we can do some tests.
	if (transition_duration >= 0.0) {
		std::cerr << "transition_duration should be positive" << std::endl;
	}

	TaskLevel *tl = new TaskLevel(task, priorityLevel);
	m_taskMotions.push_back(tl);
	addTask(tl, weight, priorityLevel);

	return true;
}
bool InverseDynamics::addJointLimitTask(TaskJointLimit & task, double weight, unsigned int priorityLevel, double transition_duration) {
	assert(weight >= 0.0);
	assert(transition_duration >= 0.0);

	// This part is not used frequently so we can do some tests.
	if (weight >= 0.0)
		std::cerr << __FILE__ << " " << __LINE__ << " "
		<< "weight should be positive" << std::endl;

	// This part is not used frequently so we can do some tests.
	if (transition_duration >= 0.0) {
		std::cerr << "transition_duration should be positive" << std::endl;
	}

	TaskLevel *tl = new TaskLevel(task, priorityLevel);
	m_taskMotions.push_back(tl);
	addTask(tl, weight, priorityLevel);

	return true;
}
bool InverseDynamics::addOperationalTask(TaskSE3Equality & task, double weight, unsigned int priorityLevel, double transition_duration)
{
	assert(weight >= 0.0);
	assert(transition_duration >= 0.0);

	// This part is not used frequently so we can do some tests.
	if (weight >= 0.0)
		std::cerr << __FILE__ << " " << __LINE__ << " "
		<< "weight should be positive" << std::endl;

	// This part is not used frequently so we can do some tests.
	if (transition_duration >= 0.0) {
		std::cerr << "transition_duration should be positive" << std::endl;
	}

	TaskLevel *tl = new TaskLevel(task, priorityLevel);
	m_taskMotions.push_back(tl);
	addTask(tl, weight, priorityLevel);

	return true;
}

bool InverseDynamics::updateTaskWeight(const std::string & task_name,
	double weight)
{
	ConstraintLevel::iterator it;
	// do not look into first priority level because weights do not matter there
	for (unsigned int i = 1; i<m_hqpData.size(); i++)
	{
		for (it = m_hqpData[i].begin(); it != m_hqpData[i].end(); it++)
		{
			if (it->second->name() == task_name)
			{
				it->first = weight;
				return true;
			}
		}
	}
	return false;
}

const HQPData & InverseDynamics::computeProblemData(double time, VectorXd q, VectorXd v)
{
	m_t = time;
	m_robot.getUpdateKinematics(q, v); 
	using namespace std;
	
	if (m_robot.type() == 0) {
		const MatrixXd & M_a = m_robot.getMassMatrix();
		const VectorXd & h_a = m_robot.getNLEtorque();

		std::vector<TaskLevel*>::iterator it;
		for (it = m_taskMotions.begin(); it != m_taskMotions.end(); it++)
		{

			const ConstraintBase & c = (*it)->task.compute(time, q, v);

			if (c.isEquality())
			{
				(*it)->constraint->matrix().leftCols(m_v) = c.matrix();
				(*it)->constraint->vector() = c.vector();
			}
			else if (c.isInequality())
			{
				(*it)->constraint->matrix().leftCols(m_v) = c.matrix();
				(*it)->constraint->lowerBound() = c.lowerBound();
				(*it)->constraint->upperBound() = c.upperBound();
			}
			else
			{
				(*it)->constraint->lowerBound().head(m_v) = c.lowerBound();
				(*it)->constraint->upperBound().head(m_v) = c.upperBound();
			}
		}
	}

	m_solutionDecoded = false;

	return m_hqpData;
}

bool InverseDynamics::decodeSolution(const HQPOutput & sol)
{
	if (m_solutionDecoded)
		return true;

	if (m_robot.type() == 0) {
		const MatrixXd & M_a = m_robot.getMassMatrix();
		const VectorXd & h_a = m_robot.getNLEtorque();
		m_dv = sol.x.head(m_v);
		m_tau = h_a;
		m_tau.noalias() += M_a*m_dv;

		m_slack.resize(7);
		m_slack = sol.x.tail(7);
		m_solutionDecoded = true;
	
		return true;
	}
	//else if (m_robot.type == 1) {
	//	const MatrixXd & M_a = m_robot.getMassMatrix().bottomRows(m_v - 6);
	////	const VectorXd & h_a = m_robot.nonLinearEffects(m_data).tail(m_v - 6);
	////	const MatrixXd & J_a = m_Jc.rightCols(m_v - 6);
	//}
	//else if (m_robot.type == 2) {
	////	const MatrixXd & M_a = m_robot.mass(m_data).bottomRows(m_v - 6);
	////	const VectorXd & h_a = m_robot.nonLinearEffects(m_data).tail(m_v - 6);
	////	const MatrixXd & J_a = m_Jc.rightCols(m_v - 6);
	//}
}

const VectorXd & InverseDynamics::getActuatorForces(const HQPOutput & sol)
{
	decodeSolution(sol);
	return m_tau;
}

const VectorXd & InverseDynamics::getAccelerations(const HQPOutput & sol)
{
	decodeSolution(sol);
	return m_dv;
}
const VectorXd & InverseDynamics::getSlack(const HQPOutput & sol) {
	decodeSolution(sol);
	return m_slack;
}
bool InverseDynamics::removeTask(const std::string & taskName, double)
{
	bool taskFound = removeFromHqpData(taskName);
	assert(taskFound);

	std::vector<TaskLevel*>::iterator it;
	for (it = m_taskMotions.begin(); it != m_taskMotions.end(); it++)
	{
		if ((*it)->task.name() == taskName)
		{
			if ((*it)->priority == 0)
			{
				if ((*it)->constraint->isEquality())
					m_eq -= (*it)->constraint->rows();
				else if ((*it)->constraint->isInequality())
					m_in -= (*it)->constraint->rows();
				else if ((*it)->constraint->isBound())
					m_bound -= (*it)->constraint->lowerBound().size();
			}
			m_taskMotions.erase(it);
			return true;
		}
	}
	return false;
}

bool InverseDynamics::removeFromHqpData(const std::string & name)
{
	bool found = false;
	for (HQPData::iterator it = m_hqpData.begin(); !found && it != m_hqpData.end(); it++)
	{
		for (ConstraintLevel::iterator itt = it->begin(); !found && itt != it->end(); itt++)
		{
			if (itt->second->name() == name)
			{
				it->erase(itt);
				return true;
			}
		}
	}
	return false;
}

