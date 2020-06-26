#include "PythonCBS.cpp"
#include <boost/python.hpp>
#include <Python.h>
namespace p = boost::python;

int main() {
	p::object Nothing("avsfd");
	PythonCBS<MapLoader> test = PythonCBS<MapLoader>(Nothing, "CBSH", 2, 300, true);
	test.search();
	return 0;
}