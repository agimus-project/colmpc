///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021-2022, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifdef PINOCCHIO_WITH_HPP_FCL

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/fcl.hpp>

#include "crocoddyl/core/utils/exception.hpp"
#include <crocoddyl/core/residual-base.hpp>
#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include <hpp/fcl/distance.h>
#include <hpp/fcl/collision_data.h>

namespace colmpc {
using namespace crocoddyl;

template <typename Scalar>
ResidualDistanceCollisionTpl<Scalar>::ResidualDistanceCollisionTpl(
    boost::shared_ptr<StateMultibody> state, const std::size_t nu,
    boost::shared_ptr<GeometryModel> geom_model,
    const pinocchio::PairIndex pair_id, const pinocchio::JointIndex joint_id)
    : Base(state, 3, nu, true, false, false),
      pin_model_(*state->get_pinocchio()),
      geom_model_(geom_model),
      pair_id_(pair_id),
      joint_id_(joint_id) {
  if (static_cast<pinocchio::FrameIndex>(geom_model->collisionPairs.size()) <=
      pair_id) {
    throw_pretty(
        "Invalid argument: "
        << "the pair index is wrong (it does not exist in the geometry model)");
  }
  if (static_cast<pinocchio::FrameIndex>(state->get_pinocchio()->njoints) <=
      joint_id) { 
    throw_pretty(
        "Invalid argument: "
        << "the joint index is wrong (it does not exist in the robot)");
  }
    
    int shape1_id = geom_model->collisionPairs[_pair_id].first;
    assert(shape1_id <= _geom_model.geometryObjects.size());

    int shape2_id = geom_model->collisionPairs[_pair_id].second;
    assert(shape1_id <= _geom_model.geometryObjects.size());

    hppfcl::distanceRequest() req;
    hppfcl::distanceResult() res; 

}



template <typename Scalar>
ResidualDistanceCollisionTpl<Scalar>::~ResidualDistanceCollisionTpl() {}

template <typename Scalar>
void ResidualDistanceCollisionTpl<Scalar>::calc(
    const boost::shared_ptr<ResidualDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &x, const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());

  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q =
      x.head(state_->get_nq());

  // computes the distance for the collision pair pair_id_
  pinocchio::updateGeometryPlacements(pin_model_, *d->pinocchio,
                                      *geom_model_.get(), d->geometry, q);


  data->r = hppfcl::distance(
    geom_model.geometryObjects[shape1_id],
    d->geometry.oMg[shape1_id],
    geom_model.geometryObjects[shape2_id],
    d->geometry.oMg[shape2_id],
    req,
    res
  );
} 

template <typename Scalar>
void ResidualModelPairCollisionTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ResidualDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &, const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());

  const std::size_t nv = state_->get_nv();

  // calculate the vector from the joint jointId to the collision p1, expressed
  // in world frame

  Matrix6xs::J1 = pinocchio::getFrameJacobian(pin_model_, *d->pinocchio, geom_model[geom_model->collisionPairs[_pair_id].first].parentFrame,
                              pinocchio::LOCAL_WORLD_ALIGNED);
  Matrix6xs::J2 = pinocchio::getFrameJacobian(pin_model_, *d->pinocchio, geom_model[geom_model->collisionPairs[_pair_id].second].parentFrame,
                              pinocchio::LOCAL_WORLD_ALIGNED);

  Vector3s::cp1 = res.getNearestPoint1();  
  Vector3s::cp2 = res.getNearestPoint2();  

  Vector3s::f1p1 = cp1 - *d->pinocchio.oMf[geom_model[geom_model->collisionPairs[_pair_id].first].parentFrame].translation;
  Matrix6xs::f1Mp1 = pinocchio::SE3::Identity();
  f1Mp1.translation = cp1;
  J1 = f1Mp1.actionInverse * J1

  Vector3s::f2p2 = cp2 - *d->pinocchio.oMf[geom_model[geom_model->collisionPairs[_pair_id].second].parentFrame].translation;
  Matrix6xs::f2Mp2 = pinocchio::SE3::Identity();
  f2Mp2.translation = cp2;
  J2 = f2Mp2.actionInverse * J2


  // calculate the Jacobia
  // compute the residual derivatives
  d->Rx.topLeftCorner(3, nv) = res.normal/res.min_distance * (J1.template topRows<3>() - J2.template topRows<3>());

    }
template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> >
ResidualModelPairCollisionTpl<Scalar>::createData(
    DataCollectorAbstract *const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this,
                                      data);
}

template <typename Scalar>
const pinocchio::GeometryModel &
ResidualModelPairCollisionTpl<Scalar>::get_geometry() const {
  return *geom_model_.get();
}

}  // namespace crocoddyl

#endif  // PINOCCHIO_WITH_HPP_FCL
