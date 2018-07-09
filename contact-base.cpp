#include "contact-base.h"

namespace HQP
{
	namespace contact
	{
		ContactBase::ContactBase(const std::string & name, RobotModel & robot) :
			m_name(name),
			m_robot(robot)
		{}

		const std::string & ContactBase::name() const
		{
			return m_name;
		}

		void ContactBase::name(const std::string & name)
		{
			m_name = name;
		}
	}
}
