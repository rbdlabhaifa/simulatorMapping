#include <boost/python.hpp>
#include <simulator/simulator.h>
#include <Python.h>

using namespace boost::python;

BOOST_PYTHON_MODULE(simulator_python)
{
    class_<Simulator, boost::noncopyable>("Simulator", init<std::string, std::string, std::string, bool, bool, std::string, bool, std::string, double, std::string, double>())
        .def("run", &Simulator::run)
        .def("simulatorRunThread", &Simulator::simulatorRunThread)
        .def("isReady", &Simulator::isReady)
        .def("getCurrentLocation", &Simulator::getCurrentLocation)
        .def("getCurrentMap", &Simulator::getCurrentMap)
        .def("command", &Simulator::command)
        .def("stop", &Simulator::stop)
        .def("setTrack", &Simulator::setTrack)
        .def("setSpeed", &Simulator::setSpeed)
        .def("getSpeed", &Simulator::getSpeed);
    ;
}
