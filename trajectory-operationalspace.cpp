#include "trajectory-operationalspace.h"
#define DEGREE	(0.01745329251994329576923690768489)
#include <iostream>


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



///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Joint cubic Trajectory ////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

TrajectoryOperationCubic::TrajectoryOperationCubic(const std::string & name)
	:TrajectoryBase(name)
{}

TrajectoryOperationCubic::TrajectoryOperationCubic(const std::string & name, const Transform3d & init_M, const Transform3d & goal_M, const double & duration, const double & stime) : TrajectoryBase(name)
{
	setGoalSample(goal_M);
	setInitSample(init_M);
	setDuration(duration);
	setStartTime(stime);

	//m_sample.resize(12, 6);
	//m_sample.pos.head<3>() = init_M.translation();
	//typedef Eigen::Matrix<double, 9, 1> Vector9;
	//m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&M.rotation()(0), 9);
}
unsigned int TrajectoryOperationCubic::size() const
{
	return 6;
}

const TrajectorySample & TrajectoryOperationCubic::operator()(double)
{
	return m_sample;
}

const TrajectorySample & TrajectoryOperationCubic::computeNext()
{

	Vector3d init_rot;
	Vector3d desired_rot;
	Vector3d cubic_tra;
	Vector3d cubic_rot_tra;
	MatrixXd cubic_rot;
	m_sample.resize(12, 6);
	typedef Eigen::Matrix<double, 9, 1> Vector9;
	Rot2euler(m_init.linear(), init_rot);
	Rot2euler(m_goal.linear(), desired_rot);
	
	if (m_time < m_stime) {
		m_sample.pos.head<3>() = m_init.translation();
		m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&m_init.rotation()(0), 9);


		return m_sample;
	}
	else if (m_time > m_stime + m_duration) {
		m_sample.pos.head<3>() = m_goal.translation();
		m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&m_goal.rotation()(0), 9);

		return m_sample;
	}
	else {
		double a0, a1, a2, a3;
		double r0, r1, r2, r3;
		for (int i = 0; i < 3; i++) {
			a0 = m_init.translation()(i);
			a1 = 0.0; //m_init.vel(i);
			a2 = 3.0 / pow(m_duration, 2) * (m_goal.translation()(i) - m_init.translation()(i)) ;
			a3 = -1.0 * 2.0 / pow(m_duration, 3) * (m_goal.translation()(i) - m_init.translation()(i));

			cubic_tra(i) = a0 + a1 * (m_time - m_stime) + a2 * pow(m_time - m_stime, 2) + a3 * pow(m_time - m_stime, 3);
		}

		for (int i = 0;i < 3;i++)
		{
			r0 = init_rot(i);
			r1 = 0.0; //m_init.vel(i);
			r2 = 3.0 / pow(m_duration, 2) * (desired_rot(i) - init_rot(i));
			r3 = -1.0 * 2.0 / pow(m_duration, 3) * (desired_rot(i) - init_rot(i));

			cubic_rot_tra(i) = r0 + r1 * (m_time - m_stime) + r2 * pow(m_time - m_stime, 2) + r3 * pow(m_time - m_stime, 3);

		}

	}
	
	cubic_rot = Rotate_with_Z(cubic_rot_tra(2))*Rotate_with_Y(cubic_rot_tra(1))*Rotate_with_X(cubic_rot_tra(0));
	m_cubic.translation() = cubic_tra;
	m_cubic.linear() = m_init.linear()	;

	

	//std::cout << GetPhi(cubic_rot, m_goal.linear()) << std::endl;

	m_sample.pos.head<3>() = m_cubic.translation();
	m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&m_cubic.rotation()(0), 9);


	return m_sample;
}

void TrajectoryOperationCubic::getLastSample(TrajectorySample & sample) const
{
	sample = m_sample;
}

bool TrajectoryOperationCubic::has_trajectory_ended() const
{
	return true;
}

void TrajectoryOperationCubic::setGoalSample(Transform3d goal_M)
{
	m_goal = goal_M;
}
void TrajectoryOperationCubic::setInitSample(Transform3d init_M)
{
	m_init = init_M;
}
void TrajectoryOperationCubic::setDuration(const double & duration)
{
	m_duration = duration;
}
void TrajectoryOperationCubic::setCurrentTime(const double & time)
{
	m_time = time;
}
void TrajectoryOperationCubic::setStartTime(const double & time)
{
	m_stime = time;
}

void TrajectoryOperationCubic::Rot2euler(Matrix3d Rot, Vector3d & angle)
{
	double beta;

	beta = -asin(Rot(2, 0));

	if (abs(beta) < 90 * DEGREE)
		beta = beta;
	else
		beta = 180 * DEGREE - beta;

	angle(0) = atan2(Rot(2, 1), Rot(2, 2) + 1E-37); //x
	angle(2) = atan2(Rot(1, 0), Rot(0, 0) + 1E-37); //z
	angle(1) = beta; //y


}

Matrix3d TrajectoryOperationCubic::Rotate_with_X(double rAngle)
{

	Matrix3d _Rotate_wth_X;

	_Rotate_wth_X(0, 0) = 1.0;
	_Rotate_wth_X(1, 0) = 0.0;
	_Rotate_wth_X(2, 0) = 0.0;

	_Rotate_wth_X(0, 1) = 0.0;
	_Rotate_wth_X(1, 1) = cos(rAngle);
	_Rotate_wth_X(2, 1) = sin(rAngle);

	_Rotate_wth_X(0, 2) = 0.0;
	_Rotate_wth_X(1, 2) = -sin(rAngle);
	_Rotate_wth_X(2, 2) = cos(rAngle);

	return(_Rotate_wth_X);

}

Matrix3d TrajectoryOperationCubic::Rotate_with_Y(double rAngle)
{
		Matrix3d _Rotate_wth_Y(3, 3);

		_Rotate_wth_Y(0, 0) = cos(rAngle);
		_Rotate_wth_Y(1, 0) = 0.0;
		_Rotate_wth_Y(2, 0) = -sin(rAngle);

		_Rotate_wth_Y(0, 1) = 0.0;
		_Rotate_wth_Y(1, 1) = 1.0;
		_Rotate_wth_Y(2, 1) = 0.0;

		_Rotate_wth_Y(0, 2) = sin(rAngle);
		_Rotate_wth_Y(1, 2) = 0.0;
		_Rotate_wth_Y(2, 2) = cos(rAngle);

		return(_Rotate_wth_Y);

}

Matrix3d TrajectoryOperationCubic::Rotate_with_Z(double rAngle)
{

	Matrix3d _Rotate_wth_Z(3, 3);

	_Rotate_wth_Z(0, 0) = cos(rAngle);
	_Rotate_wth_Z(1, 0) = sin(rAngle);
	_Rotate_wth_Z(2, 0) = 0.0;

	_Rotate_wth_Z(0, 1) = -sin(rAngle);
	_Rotate_wth_Z(1, 1) = cos(rAngle);
	_Rotate_wth_Z(2, 1) = 0.0;

	_Rotate_wth_Z(0, 2) = 0.0;
	_Rotate_wth_Z(1, 2) = 0.0;
	_Rotate_wth_Z(2, 2) = 1.0;

	return(_Rotate_wth_Z);

}

Vector3d TrajectoryOperationCubic::GetPhi(Matrix3d Rot, Matrix3d Rotd)
{
	Vector3d phi;
	Vector3d s[3], v[3], w[3];

	for (int i = 0; i < 3; i++) {
		v[i] = Rot.block(0, i, 3, 1);
		w[i] = Rotd.block(0, i, 3, 1);
		s[i] = v[i].cross(w[i]);
	}
	phi = s[0] + s[1] + s[2];
	phi = -0.5* phi;

	return phi;
}