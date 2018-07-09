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

 
