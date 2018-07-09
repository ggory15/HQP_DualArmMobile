#ifndef __HQP_CONTAINER__
#define __HQP_CONTAINER__

#include <vector>
#include <Eigen/StdVector>

namespace HQP
{
	namespace container
	{

		template<typename T>
		struct aligned_vector : public std::vector<T, Eigen::aligned_allocator<T> >
		{
			typedef ::std::vector<T, Eigen::aligned_allocator<T> > vector_base;
			typedef T value_type;
			typedef typename vector_base::allocator_type allocator_type;
			typedef typename vector_base::size_type size_type;
			typedef typename vector_base::iterator iterator;

			explicit aligned_vector(const allocator_type & a = allocator_type()) : vector_base(a) {}
			template<typename InputIterator>
			aligned_vector(InputIterator first, InputIterator last, const allocator_type& a = allocator_type())
				: vector_base(first, last, a) {}
			aligned_vector(const aligned_vector & c) : vector_base(c) {}

			explicit aligned_vector(size_type num, const value_type & val = value_type())
				: vector_base(num, val) {}

			aligned_vector(iterator start, iterator end) : vector_base(start, end) {}
			aligned_vector & operator=(const aligned_vector& x)
			{
				vector_base::operator=(x); return *this;
			}

			vector_base & base() { return *static_cast<vector_base*>(this); }
			const vector_base & base() const { return *static_cast<const vector_base*>(this); }

		}; // struct aligned_vector



		template<class T>
		bool operator==(const aligned_vector<T>& lhs, const aligned_vector<T>& rhs)
		{
			typedef typename aligned_vector<T>::vector_base vector_base;
			return *static_cast<const vector_base*>(&lhs) == *static_cast<const vector_base*>(&rhs);
		}

	} // namespace container
} // namespace HQP



#endif // __HQP_CONTAINER__