#ifndef __HQP__MATH__SE3__
#define __HQP__MATH__SE3__

#include "fwd.h"
#include "motion.h"

namespace HQP {
	Eigen::Matrix3d	alphaSkew(const double & s, const Vector3d & v);
	Eigen::VectorXd log3(const MatrixXd & R, double & theta);
	Eigen::VectorXd log3(const MatrixXd & R);
	MotionVector<double> log6(Transform3d & M);
}
#endif __HQP__MATH__SE3__

