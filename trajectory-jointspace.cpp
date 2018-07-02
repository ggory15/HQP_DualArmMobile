#include "trajectory-jointspace.h"

using namespace HQP::trajectories;

TrajectoryJointConstant::TrajectoryJointConstant(const std::string & name)
    :TrajectoryBase(name)
{}

TrajectoryJointConstant::TrajectoryJointConstant(const std::string & name, Cref_vectorXd ref)
    :TrajectoryBase(name)
{
    setReference(ref);
}

void TrajectoryJointConstant::setReference(Cref_vectorXd ref)
{
    m_sample.pos = ref;
    m_sample.vel.setZero(ref.size());
    m_sample.acc.setZero(ref.size());
}

unsigned int TrajectoryJointConstant::size() const
{
    return (unsigned int)m_sample.pos.size();
}

const TrajectorySample & TrajectoryJointConstant::operator()(double )
{
    return m_sample;
}

const TrajectorySample & TrajectoryJointConstant::computeNext()
{
    return m_sample;
}

void TrajectoryJointConstant::getLastSample(TrajectorySample & sample) const
{
    sample = m_sample;
}

bool TrajectoryJointConstant::has_trajectory_ended() const
{
    return true;
}

///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Joint cubic Trajectory ////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
TrajectoryJointCubic::TrajectoryJointCubic(const std::string & name)
	:TrajectoryBase(name)
{}

TrajectoryJointCubic::TrajectoryJointCubic(const std::string & name, Cref_vectorXd init, Cref_vectorXd goal, const double & duration, const double & stime)
	: TrajectoryBase(name)
{
	//setReference(init);
	setGoalSample(goal);
	setInitSample(init);
	setDuration(duration);
	setStartTime(stime);


}

void TrajectoryJointCubic::setGoalSample(Cref_vectorXd ref)
{
	m_goal.pos = ref;
	m_goal.vel.setZero(ref.size());
	m_goal.acc.setZero(ref.size());
}
void TrajectoryJointCubic::setInitSample(Cref_vectorXd ref)
{
	m_init.pos = ref;
	m_init.vel.setZero(ref.size());
	m_init.acc.setZero(ref.size());
}
void TrajectoryJointCubic::setDuration(const double & duration)
{
	m_duration = duration;
}
unsigned int TrajectoryJointCubic::size() const
{
	return (unsigned int)m_init.pos.size();
}

const TrajectorySample & TrajectoryJointCubic::operator()(double)
{
	return m_init;
}
void TrajectoryJointCubic::setCurrentTime(const double & time)
{
	m_time = time;
}
void TrajectoryJointCubic::setStartTime(const double & time)
{
	m_stime = time;
}

const TrajectorySample & TrajectoryJointCubic::computeNext()
{
	unsigned int size = m_init.pos.size();
	m_sample.pos.resize(size);
	m_sample.vel.resize(size);
	m_sample.acc.resize(size);
	//assert(time > m_stime && time < m_stime + m_duration);
	if (m_time < m_stime)
		return m_init;
	else if (m_time > m_stime + m_duration)
		return m_goal;
	else {
		double a0, a1, a2, a3;
		for (int i = 0; i < m_init.pos.size(); i++) {
			a0 = m_init.pos(i);
			a1 = m_init.vel(i);
			a2 = 3.0 / pow(m_duration, 2) * (m_goal.pos(i) - m_init.pos(i)) - 2.0 / m_duration * m_init.vel(i) - 1 / m_duration * m_goal.vel(i);
			a3 = -1.0 * 2.0 / pow(m_duration, 3) * (m_goal.pos(i) - m_init.pos(i)) + 1.0 / pow(m_duration,2) * (m_init.vel(i) + m_goal.vel(i));

			m_sample.pos(i) = a0 + a1 * (m_time - m_stime) + a2 * pow(m_time - m_stime, 2) + a3 * pow(m_time - m_stime, 3);
			m_sample.vel(i) = a1 + a2 * 2.0 * (m_time - m_stime) + 3.0 * a3 * pow((m_time - m_stime), 2);
			m_sample.acc(i) = 2.0 * a2 + 6.0 * a3 * (m_time - m_stime);
		}
	}
	return m_sample;
}

void TrajectoryJointCubic::getLastSample(TrajectorySample & sample) const
{
	sample = m_goal;
}

bool TrajectoryJointCubic::has_trajectory_ended() const
{
	return true;
}

 
