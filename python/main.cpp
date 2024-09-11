#include "colmpc/python.hpp"

BOOST_PYTHON_MODULE(colmpc) {
  namespace bp = boost::python;

  bp::import("pinocchio");
  bp::import("crocoddyl");
  // Enabling eigenpy support, i.e. numpy/eigen compatibility.
  eigenpy::enableEigenPy();
  eigenpy::enableEigenPySpecific<Eigen::VectorXi>();
  colmpc::python::exposeResidualDistanceCollision();
}
