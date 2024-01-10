///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021-2022, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifdef PINOCCHIO_WITH_HPP_FCL

#include "colmpc/residual-distance-collision.hpp"

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
  if (static_cast<pinocchio::FrameIndex>(geom_model_->collisionPairs.size()) <=
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

  int shape1_id = geom_model_->collisionPairs[pair_id].first;
  assert(shape1_id <= geom_model_.geometryObjects.size());

  int shape2_id = geom_model_->collisionPairs[pair_id].second;
  assert(shape1_id <= geom_model_.geometryObjects.size());

  hpp::fcl::DistanceRequest req = hpp::fcl::DistanceRequest();

  // hpp::fcl::DistanceRequest req = hpp::fcl::DistanceRequest(bool
  // enable_nearest_points_ = true) ;
  hpp::fcl::DistanceResult res = hpp::fcl::DistanceResult();
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

  data->r = hpp::fcl::distance(geom_model_->geometryObjects[shape1_id],
                               d->geometry.oMg[shape1_id],
                               geom_model_->geometryObjects[shape2_id],
                               d->geometry.oMg[shape2_id], req, res);
}

template <typename Scalar>
void ResidualDistanceCollisionTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ResidualDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &, const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());

  const std::size_t nv = state_->get_nv();

  // calculate the jacobians of the frames of the collision pairs

  Matrix6xs::J1 = pinocchio::getFrameJacobian(
      pin_model_, *d->pinocchio,
      geom_model_->geometryObjects[geom_model_->collisionPairs[pair_id_].first]
          .parentFrame,
      pinocchio::LOCAL_WORLD_ALIGNED);
  Matrix6xs::J2 = pinocchio::getFrameJacobian(
      pin_model_, *d->pinocchio,
      geom_model_->geometryObjects[geom_model_->collisionPairs[pair_id_].second]
          .parentFrame,
      pinocchio::LOCAL_WORLD_ALIGNED);

  // getting the nearest points belonging to the collision shapes
  Vector3s::cp1 = res.nearest_points[0];
  Vector3s::cp2 = res.nearest_points[1];

  // Transport the jacobian of frame 1 into the jacobian associated to cp1
  // Vector from frame 1 center to p1
  Vector3s::f1p1 =
      cp1 -
      *d->pinocchio
           .oMf[geom_model_
                    ->geometryObjects[geom_model_->collisionPairs[pair_id_]
                                          .first]
                    .parentFrame]
           .translation;
  Matrix6xs::f1Mp1 = pinocchio::SE3::Identity();
  f1Mp1.translation = cp1;
  J1 = f1Mp1.actionInverse * J1;

  // Transport the jacobian of frame 2 into the jacobian associated to cp2
  // Vector from frame 2 center to p2
  Vector3s::f2p2 =
      cp2 -
      *d->pinocchio
           .oMf[geom_model_
                    ->geometryObjects[geom_model_->collisionPairs[pair_id_]
                                          .second]
                    .parentFrame]
           .translation;
  Matrix6xs::f2Mp2 = pinocchio::SE3::Identity();
  f2Mp2.translation = cp2;
  J2 = f2Mp2.actionInverse * J2;

  // calculate the Jacobia
  // compute the residual derivatives
  d->Rx.topLeftCorner(3, nv) =
      res.normal / res.min_distance *
      (J1.template topRows<3>() - J2.template topRows<3>());
}
template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> >
ResidualDistanceCollisionTpl<Scalar>::createData(
    DataCollectorAbstract *const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this,
                                      data);
}

template <typename Scalar>
const pinocchio::GeometryModel &
ResidualDistanceCollisionTpl<Scalar>::get_geometry() const {
  return *geom_model_.get();
}

}  // namespace colmpc

#endif  // PINOCCHIO_WITH_HPP_FCL
