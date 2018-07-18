#include "trajectories/trajectory-operationalspace.h"

using namespace HQP::trajectories;

TrajectoryOperationConstant::TrajectoryOperationConstant(const std::string & name)
    :TrajectoryBase(name)
{}

TrajectoryOperationConstant::TrajectoryOperationConstant(const std::string & name, const Transform3d & M) :TrajectoryBase(name)
{
    m_sample.resize(12, 6);
	m_sample.pos.head<3>() = M.translation();
	typedef Eigen::Matrix<double, 9, 1> Vector9;
	m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&M.rotation()(0), 9);
}
void TrajectoryOperationConstant::setReference(const Transform3d ref) {
	m_sample.resize(12, 6);
	m_sample.pos.head<3>() = ref.translation();
	typedef Eigen::Matrix<double, 9, 1> Vector9;
	m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&ref.rotation()(0), 9);
}
unsigned int TrajectoryOperationConstant::size() const
{
    return 6;
}

const TrajectorySample & TrajectoryOperationConstant::operator()(double )
{
    return m_sample;
}

const TrajectorySample & TrajectoryOperationConstant::computeNext()
{
    return m_sample;
}

void TrajectoryOperationConstant::getLastSample(TrajectorySample & sample) const
{
    sample = m_sample;
}

bool TrajectoryOperationConstant::has_trajectory_ended() const
{
    return true;
}
