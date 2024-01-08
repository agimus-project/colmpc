#include "sobec/py2cpp.hpp"

#include <crocoddyl/core/utils/exception.hpp>
#include <sobec/walk-without-think/mpc.hpp>

namespace bp = boost::python;

namespace sobec {
ShootingProblemPtr initShootingProblem(const char* fileName) {
  try {
    Py_Initialize();
    bp::object main_module = bp::import("__main__");
    bp::object main_namespace = main_module.attr("__dict__");
    bp::exec_file(fileName, main_namespace);
    return bp::extract<ShootingProblemPtr>(main_namespace["problem"]);
  } catch (bp::error_already_set&) {
    PyErr_Print();
    throw_pretty(__FILE__ ": python error")
  }
}
MPCWalkPtr initMPCWalk(const char* fileName) {
  try {
    Py_Initialize();
    bp::object main_module = bp::import("__main__");
    bp::object main_namespace = main_module.attr("__dict__");
    bp::exec_file(fileName, main_namespace);
    return bp::extract<MPCWalkPtr>(main_namespace["mpc"]);
  } catch (bp::error_already_set&) {
    PyErr_Print();
    throw_pretty(__FILE__ ": python error")
  }
}

bool reprProblem(ShootingProblemPtr problem) {
  bool ret = true;
  Py_Initialize();
  bp::object main_module = bp::import("__main__");
  bp::object main_namespace = main_module.attr("__dict__");

  try {
    bp::import("sobec");
    bp::object pyproblem(problem);
    main_module.attr("problem_from_cpp") = pyproblem;
    bp::exec(
        "from sobec.walk.miscdisp import printReprProblem; "
        "printReprProblem(problem_from_cpp)",
        main_namespace);
  } catch (bp::error_already_set&) {
    PyErr_Print();
    ret = false;
  }
  return ret;
}
}  // namespace sobec
