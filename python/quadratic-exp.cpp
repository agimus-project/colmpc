///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2024, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "colmpc/quadratic-exp.hpp"
#include "colmpc/python.hpp"

namespace colmpc {
namespace python {

namespace bp = boost::python;

void exposeActivationModelQuadExp() {
  bp::register_ptr_to_python<boost::shared_ptr<ActivationModelQuadExp> >();

  bp::class_<ActivationModelQuadExp, bp::bases<crocoddyl::ActivationModelAbstract> >(
      "ActivationModelQuadExp",
      "Quadratic activation model.\n\n"
      "A quadratic  action describes a quadratic  function that "
      "depends on the residual, i.e.\n"
      "exp(-||r||^2 / alpha).",
      bp::init<int, double>(bp::args("self", "nr", "alpha"),
                            "Initialize the activation model.\n\n"
                            ":param nr: dimension of the cost-residual vector"
                            "param alpha: width of quadratic basin near zero"))
      .def("calc", &ActivationModelQuadExp::calc,
           bp::args("self", "data", "r"),
           "Compute the exp(-||r||^2 / alpha).\n\n"
           ":param data: activation data\n"
           ":param r: residual vector")
      .def("calcDiff", &ActivationModelQuadExp::calcDiff,
           bp::args("self", "data", "r"),
           "Compute the derivatives of a quadratic  function.\n\n"
           "Note that the Hessian is constant, so we don't write again this "
           "value.\n"
           ":param data: activation data\n"
           ":param r: residual vector \n")
      .def("createData", &ActivationModelQuadExp::createData,
           bp::args("self"), "Create the quadratic  activation data.\n\n")
      .add_property(
          "alpha", bp::make_function(&ActivationModelQuadExp::get_alpha),
          bp::make_function(&ActivationModelQuadExp::set_alpha), "alpha");
}

}  // namespace python
}  // namespace colmpc