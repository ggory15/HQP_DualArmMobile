#ifndef __HQP__MATH__
#define __HQP__MATH__

#include "fwd.h"

namespace HQP {
	template<typename Scalar>
	class MotionVector
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		typedef Scalar ScalarType;
		typedef typename ::Eigen::Matrix<Scalar, 6, 1> MatrixType;
		typedef const MatrixType ConstMatrixType;
		typedef ::Eigen::Block<MatrixType, 3, 1> AngularType;
		typedef const ::Eigen::Block<ConstMatrixType, 3, 1> ConstAngularType;
		typedef ::Eigen::Block<MatrixType, 3, 1> LinearType;
		typedef const ::Eigen::Block<ConstMatrixType, 3, 1> ConstLinearType;

		MotionVector(){}
		MotionVector(const VectorXd v, const VectorXd w) {
			data.head<3>() = v;
			data.tail<3>() = w;
		}
		template<typename OtherDerived>
		MotionVector(const ::Eigen::MatrixBase<OtherDerived>& other) :
			data(other)
		{
		}

		virtual ~MotionVector()
		{
		}

		AngularType angular()
		{
			return data.template segment<3>(3);
		}

		ConstAngularType angular() const
		{
			return data.template segment<3>(3);
		}

		MotionVector cross(const MotionVector& other) const
		{
			MotionVector res;
			res.angular() = angular().cross(other.angular());
			res.linear() = angular().cross(other.linear()) + linear().cross(other.angular());
			return res;
		}
		LinearType linear()
		{
			return data.template segment<3>(0);
		}

		ConstLinearType linear() const
		{
			return data.template segment<3>(0);
		}

		ConstMatrixType& vector() const
		{
			return data;
		}

		template<typename OtherDerived>
		MotionVector& operator=(const ::Eigen::MatrixBase<OtherDerived>& other)
		{
			data = other;
			return *this;
		}

		MotionVector operator+(const MotionVector& other) const
		{
			MotionVector res;
			res.angular() = angular() + other.angular();
			res.linear() = linear() + other.linear();
			return res;
		}

		MotionVector operator-(const MotionVector& other) const
		{
			MotionVector res;
			res.angular() = angular() - other.angular();
			res.linear() = linear() - other.linear();
			return res;
		}

		template<typename OtherScalar>
		MotionVector operator*(const OtherScalar& other) const
		{
			MotionVector res;
			res.angular() = angular() * other;
			res.linear() = linear() * other;
			return res;
		}

		template<typename OtherScalar>
		MotionVector operator/(const OtherScalar& other) const
		{
			MotionVector res;
			res.angular() = angular() / other;
			res.linear() = linear() / other;
			return res;
		}

		void setZero()
		{
			angular().setZero();
			linear().setZero();
		}
		MotionVector actInv(const Transform3d& m) const {
			return MotionVector(m.linear().transpose()*(linear() - m.translation().cross(angular())), m.linear().transpose()*angular());
		}
	protected:

	private:
		MatrixType data;
	};
}


#endif

