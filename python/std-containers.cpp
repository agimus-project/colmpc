///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <eigenpy/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/force.hpp>

#if EIGENPY_VERSION_AT_LEAST(2, 8, 0)
#include <eigenpy/std-map.hpp>
#include <eigenpy/std-vector.hpp>
#else
#include <pinocchio/bindings/python/utils/std-map.hpp>
#include <pinocchio/bindings/python/utils/std-vector.hpp>
#endif

namespace sobec {
namespace python {

namespace bp = boost::python;

#if EIGENPY_VERSION_AT_LEAST(2, 8, 0)
using eigenpy::StdVectorPythonVisitor;
#elif PINOCCHIO_VERSION_AT_LEAST(2, 9, 2)
using pinocchio::python::StdVectorPythonVisitor;
#else
// the template parameter used to be the value_type.
template <typename vector_type>
struct StdVectorPythonVisitor : pinocchio::python::StdVectorPythonVisitor<
                                    typename vector_type::value_type> {};
#endif

void exposeStdContainers() {
  typedef std::vector<std::string> std_vec_str_t;
  StdVectorPythonVisitor<std_vec_str_t>::expose("StdVec_StdString");
  StdVectorPythonVisitor<std::vector<pinocchio::Force>>::expose(
      "StdVectorPinocchioForce_");
  StdVectorPythonVisitor<std::vector<std::vector<pinocchio::Force>>>::expose(
      "StdVectorStdVectorPinocchioForce_");

  using frame_frame_map_t =
      std::map<pinocchio::FrameIndex, pinocchio::FrameIndex>;
  bp::class_<frame_frame_map_t>(
      "StdMapPinocchioFrameIndexToPinocchioFrameIndex_")
      .def(bp::map_indexing_suite<frame_frame_map_t, true>())
      // .def(ep::details::overload_base_get_item_for_std_map<std::map<pinocchio::FrameIndex,
      // pinocchio::FrameIndex>>())
      ;

  using StdVecVectorXd = std::vector<Eigen::VectorXd>;
  using pair_vec_vec_t = std::pair<StdVecVectorXd, StdVecVectorXd>;
  bp::class_<pair_vec_vec_t>("StdPair_StdVector_EigenVectorXd")
      .def_readwrite("first", &pair_vec_vec_t::first)
      .def_readwrite("second", &pair_vec_vec_t::second);
}

}  // namespace python
}  // namespace sobec
