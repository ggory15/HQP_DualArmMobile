#include "utils.h"

#define SINCOS(a,sa,ca) (*sa) = std::sin(a); (*ca) = std::cos(a)

namespace HQP {
	Eigen::Matrix3d	alphaSkew(const double & s, const Vector3d & v)
	{
		Eigen::Matrix3d m;
		m(0, 0) = 0;  m(0, 1) = -v(2) * s;   m(0, 2) = v(1) * s;
		m(1, 0) = -m(0, 1);  m(1, 1) = 0;   m(1, 2) = -v(0) * s;
		m(2, 0) = -m(0, 2);  m(2, 1) = -m(1, 2);   m(2, 2) = 0;

		return m;
	}

	Eigen::VectorXd log3(const MatrixXd & R, double & theta) {
		//EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(R, Eigen::Matrix3d);
		assert(R.cols() == 3);
		assert(R.rows() == 3);

		VectorXd res(3);
		const double tr = R.trace();
		if (tr > 3)       theta = 0; // acos((3-1)/2)
		else if (tr < -1) theta = 3.1416; // acos((-1-1)/2)
		else              theta = acos((tr - 1) / 2);

		assert(theta == theta); // theta != NaN
								// From runs of hpp-constraints/tests/logarithm.cc: 1e-6 is too small.

		if (theta < 3.1416 - 1e-2) {
			const double t = ((theta > 1e-6) ? theta / sin(theta) : 1) / 2;
			res(0) = t * (R(2, 1) - R(1, 2));
			res(1) = t * (R(0, 2) - R(2, 0));
			res(2) = t * (R(1, 0) - R(0, 1));
		}
		else {
			const double cphi = cos(theta - 3.1416);
			const double beta = theta*theta / (1 + cphi);
			Vector3d tmp((R.diagonal().array() + cphi) * beta);

			res(0) = (R(2, 1) > R(1, 2) ? 1 : -1) * (tmp[0] > 0 ? sqrt(tmp[0]) : 0);
			res(1) = (R(0, 2) > R(2, 0) ? 1 : -1) * (tmp[1] > 0 ? sqrt(tmp[1]) : 0);
			res(2) = (R(1, 0) > R(0, 1) ? 1 : -1) * (tmp[2] > 0 ? sqrt(tmp[2]) : 0);
		}

		return res;
	}
	Eigen::VectorXd log3(const MatrixXd & R) {
		double theta = 0.0;
		return log3(R.derived(), theta);
	}
	MotionVector<double> log6(Transform3d & M)
	{
		MotionVector<double> a;

		MatrixXd R = M.linear();
		VectorXd p = M.translation();

		double t;
		VectorXd w = log3(R, t);

		const double t2 = t*t;
		double alpha, beta;

		if (std::fabs(t) < 1e-4) {
			alpha = 1 - t2 / 12 - t2*t2 / 720;
			beta = 1. / 12 + t2 / 720;
		}
		else {
			double st, ct;
			SINCOS(t, &st, &ct);
			alpha = t*st / (2 * (1 - ct));
			beta = 1 / t2 - st / (2 * t*(1 - ct));
		}

		return MotionVector<double>(alpha * p - alphaSkew(0.5, w) * p + beta * w.dot(p) * w, w);
	}
}
