#include "colmpc/residual-distance-collision.hpp"

#include "colmpc/python.hpp"

namespace colmpc {
namespace python {

namespace bp = boost::python;

void exposeResidualDistanceCollision() {
  bp::register_ptr_to_python<
      boost::shared_ptr<ResidualDistanceCollisionTpl<double>>>();

  bp::class_<ResidualDistanceCollisionTpl<double>,
             bp::bases<ResidualModelAbstractTpl<double>>>(
      "ResidualDistanceCollision",
      bp::init<boost::shared_ptr<StateMultibodyTpl<double>>, const std::size_t,
               boost::shared_ptr<pinocchio::GeometryModel>,
               const pinocchio::PairIndex, const pinocchio::JointIndex>(
          bp::args("self", "state", "nu", "geom_model", "pair_id", "joint_id"),
          "Initialize the residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param geom_model: Pinocchio geometry model containing the "
          "collision pair\n"
          ":param pair_id: Index of the collision pair in the geometry model\n"
          ":param joint_id: Index of the nearest joint on which the collision "
          "link is attached"))
      .def("calc", &ResidualDistanceCollisionTpl<double>::calc,
           bp::args("self", "data", "x", "u"),
           "Compute the residual.\n\n"
           ":param data: residual data\n"
           ":param x: time-discrete state vector\n"
           ":param u: time-discrete control input")
      .def("calcDiff", &ResidualDistanceCollisionTpl<double>::calcDiff,
           bp::args("self", "data", "x", "u"),
           "Compute the Jacobians of the residual.\n\n"
           "It assumes that calc has been run first.\n"
           ":param data: action data\n"
           ":param x: time-discrete state vector\n"
           ":param u: time-discrete control input\n")
      .def("createData", &ResidualDistanceCollisionTpl<double>::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for a predefined residual.\n"
           ":param data: shared data\n"
           ":return residual data.");

  bp::register_ptr_to_python<
      boost::shared_ptr<ResidualDataDistanceCollisionTpl<double>>>();

  bp::class_<ResidualDataDistanceCollisionTpl<double>,
             bp::bases<ResidualDataAbstractTpl<double>>>(
      "ResidualDataDistanceCollisionTpl",
      "Data for vel collision residual.\n\n",
      bp::init<ResidualDistanceCollisionTpl<double>*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create vel collision residual data.\n\n"
          ":param model: pair collision residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3>>()])
      .add_property(
          "geometry",
          bp::make_getter(&ResidualDataDistanceCollisionTpl<double>::geometry,
                          bp::return_internal_reference<>()),
          "pinocchio geometry data")
      .add_property(
          "pinocchio",
          bp::make_getter(&ResidualDataDistanceCollisionTpl<double>::pinocchio,
                          bp::return_internal_reference<>()),
          "pinocchio data");
}

}  // namespace python
}  // namespace colmpc
