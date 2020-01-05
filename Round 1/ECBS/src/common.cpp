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

std::ostream& operator<<(std::ostream& os, const Path& path)
{
	if (path.empty())
	{
		os << -1 << ",";
		return os;
	}
	Path::const_iterator state = path.begin();
    for (int t = 0; t <= path.back().timestep; t++)
    {
		if (state->timestep == t)
		{
			os << state->id << ",";
			++state;
		}
		else
			os << -2 << ",";
    }
    return os;
}