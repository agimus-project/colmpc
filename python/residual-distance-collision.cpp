#include "colmpc/residual-distance-collision.hpp"

#include "colmpc/python.hpp"

namespace colmpc {
namespace python {

namespace bp = boost::python;

void exposeResidualDistanceCollision() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualDistanceCollision>>();

  bp::class_<ResidualDistanceCollision, bp::bases<ResidualModelAbstract>>(
      "ResidualDistanceCollision",
      bp::init<boost::shared_ptr<StateMultibody>, const std::size_t,
               boost::shared_ptr<pinocchio::GeometryModel>,
               const pinocchio::PairIndex>(
          bp::args("self", "state", "nu", "geom_model", "pair_id"),
          "Initialize the residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param geom_model: Pinocchio geometry model containing the "
          "collision pair\n"
          ":param pair_id: Index of the collision pair in the geometry "
          "model\n"))
      .def("calc", &ResidualDistanceCollision::calc,
           bp::args("self", "data", "x", "u"),
           "Compute the residual.\n\n"
           ":param data: residual data\n"
           ":param x: time-discrete state vector\n"
           ":param u: time-discrete control input")
      .def("calcDiff", &ResidualDistanceCollision::calcDiff,
           bp::args("self", "data", "x", "u"),
           "Compute the Jacobians of the residual.\n\n"
           "It assumes that calc has been run first.\n"
           ":param data: action data\n"
           ":param x: time-discrete state vector\n"
           ":param u: time-discrete control input\n")
      .def("createData", &ResidualDistanceCollision::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for a predefined residual.\n"
           ":param data: shared data\n"
           ":return residual data.");

  bp::register_ptr_to_python<
      boost::shared_ptr<ResidualDataDistanceCollision>>();

  bp::class_<ResidualDataDistanceCollision, bp::bases<ResidualDataAbstract>>(
      "ResidualDataDistanceCollisionTpl",
      "Data for vel collision residual.\n\n",
      bp::init<ResidualDistanceCollision*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create vel collision residual data.\n\n"
          ":param model: pair collision residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3>>()])
      .add_property("geometry",
                    bp::make_getter(&ResidualDataDistanceCollision::geometry,
                                    bp::return_internal_reference<>()),
                    "pinocchio geometry data")
      .add_property("pinocchio",
                    bp::make_getter(&ResidualDataDistanceCollision::pinocchio,
                                    bp::return_internal_reference<>()),
                    "pinocchio data");
}

}  // namespace python
}  // namespace colmpc
