#include "common.h"

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << get<0>(constraint) << ", " << get<1>(constraint)
		<< ", " << get<2>(constraint);
	if (get<3>(constraint))
		os << ", positive>";
	else
		os << ", negative>";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
	os << "<" << get<0>(conflict) << ", " << get<1>(conflict) << ","
		<< get<2>(conflict) << ", " << get<3>(conflict) << ", " << get<4>(conflict) << ">";
	return os;
}
