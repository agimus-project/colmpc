///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2024, CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define PINOCCHIO_WITH_HPP_FCL

#ifndef COLMPC_RESIDUAL_VELOCITY_AVOIDANCE_HXX_
#define COLMPC_RESIDUAL_VELOCITY_AVOIDANCE_HXX_
#ifdef PINOCCHIO_WITH_HPP_FCL

#include "colmpc/residual-velocity-avoidance.hpp"

namespace colmpc {
using namespace crocoddyl;

template <typename Scalar>
ResidualModelVelocityAvoidanceTpl<Scalar>::ResidualModelVelocityAvoidanceTpl(
    boost::shared_ptr<StateMultibody> state, const std::size_t nu,
    boost::shared_ptr<GeometryModel> geom_model,
    const pinocchio::PairIndex pair_id, const Scalar di, const Scalar ds,
    const Scalar ksi)
    : Base(state, 1, nu, true, true, true),
      pin_model_(*state->get_pinocchio()),
      geom_model_(geom_model),
      pair_id_(pair_id),
      di_(di),
      ds_(ds),
      ksi_(ksi) {
  if (static_cast<pinocchio::FrameIndex>(geom_model_->collisionPairs.size()) <=
      pair_id) {
    throw_pretty(
        "Invalid argument: "
        << "the pair index is wrong (it does not exist in the geometry model)");
  }
}

template <typename Scalar>
ResidualModelVelocityAvoidanceTpl<
    Scalar>::~ResidualModelVelocityAvoidanceTpl() {}

template <typename Scalar>
void ResidualModelVelocityAvoidanceTpl<Scalar>::calc(
    const boost::shared_ptr<ResidualDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &, const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());

  // clear the hppfcl results
  d->res.clear();

  const auto &cp = geom_model_->collisionPairs[pair_id_];
  const auto &geom_1 = geom_model_->geometryObjects[cp.first];
  const auto &geom_2 = geom_model_->geometryObjects[cp.second];
  const pinocchio::Model::JointIndex joint_id_1 = geom_1.parentJoint;
  const pinocchio::Model::JointIndex joint_id_2 = geom_2.parentJoint;

  // Update geometry placement
  if (joint_id_1 > 0) {
    d->oMg_id_1 = d->pinocchio->oMi[joint_id_1] * geom_1.placement;
  } else {
    d->oMg_id_1 = geom_1.placement;
  }

  if (joint_id_2 > 0) {
    d->oMg_id_2 = d->pinocchio->oMi[joint_id_2] * geom_2.placement;
  } else {
    d->oMg_id_2 = geom_2.placement;
  }

  // compute distance between geometries
  d->distance = hpp::fcl::distance(
      geom_1.geometry.get(), toFclTransform3f(d->oMg_id_1),
      geom_2.geometry.get(), toFclTransform3f(d->oMg_id_2), d->req, d->res);

  // compute velocity
  d->m1 =
      pinocchio::getFrameVelocity(pin_model_, *d->pinocchio, geom_1.parentFrame,
                                  pinocchio::LOCAL_WORLD_ALIGNED);
  d->m2 =
      pinocchio::getFrameVelocity(pin_model_, *d->pinocchio, geom_2.parentFrame,
                                  pinocchio::LOCAL_WORLD_ALIGNED);

  const Vector3s &x1 = d->res.nearest_points[0];
  const Vector3s &x2 = d->res.nearest_points[1];
  const Vector3s &c1 = d->oMg_id_1.translation();
  const Vector3s &c2 = d->oMg_id_2.translation();

  Vector3s Lc = x2 - x1;
  const Vector3s Lr1 = c1.transpose() * pinocchio::skew(Lc) +
                       x2.transpose() * pinocchio::skew(x1);

  // Negate Lc inverting order of substraction to x1 - x2
  Lc.noalias() = -1.0 * Lc;
  const Vector3s Lr2 = c2.transpose() * pinocchio::skew(Lc) +
                       x1.transpose() * pinocchio::skew(x2);

  const Scalar Ldot = Lc.dot(d->m1.linear() - d->m2.linear()) +
                      Lr1.dot(d->m1.angular()) + Lr2.dot(d->m2.angular());
  d->r[0] = (Ldot / d->distance) + ksi_ * (d->distance - ds_) / (di_ - ds_);
}

template <typename Scalar>
void ResidualModelVelocityAvoidanceTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ResidualDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &x, const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());

  const std::size_t nq = pin_model_.nq;
  d->q = x.head(nq);
  d->v = x.tail(nq);

  // Create labels for geometries
  const auto &cp = geom_model_->collisionPairs[pair_id_];
  const auto &geom_1 = geom_model_->geometryObjects[cp.first];
  const auto &geom_2 = geom_model_->geometryObjects[cp.second];
  // Crate labels for points
  const Vector3s &x1 = d->res.nearest_points[0];
  const Vector3s &x2 = d->res.nearest_points[1];
  const Vector3s &c1 = d->oMg_id_1.translation();
  const Vector3s &c2 = d->oMg_id_2.translation();
  // Create labels for velocities
  const Vector3s &v1 = d->m1.linear();
  const Vector3s &v2 = d->m2.linear();
  const Vector3s &w1 = d->m1.angular();
  const Vector3s &w2 = d->m2.angular();
  // Convert sphere representation to a diagonal matrix
  const DiagonalMatrix3s D1 =
      std::static_pointer_cast<hpp::fcl::Ellipsoid>(geom_1.geometry)
          ->radii.asDiagonal();
  const DiagonalMatrix3s D2 =
      std::static_pointer_cast<hpp::fcl::Ellipsoid>(geom_2.geometry)
          ->radii.asDiagonal();
  // Store rotations of geometries in matrices
  const Matrix3s &R1 = d->oMg_id_1.rotation();
  const Matrix3s &R2 = d->oMg_id_2.rotation();

  const Matrix3s A1 = R1 * D1 * R1.transpose();
  const Matrix3s A2 = R2 * D2 * R2.transpose();

  // Precompute differences as they are often used;
  const Vector3s x_diff = x1 - x2;
  const Vector3s x1_c1_diff = x1 - c1;
  const Vector3s x2_c2_diff = c2 - c2;
  // Precompute cross products as they are used many times
  const Vector3s x_diff_cross_x1_c1_diff = x_diff.cross(x1_c1_diff);
  const Vector3s x_diff_cross_x2_c2_diff = x_diff.cross(x2_c2_diff);

  const Scalar sol_lam1 = -x1_c1_diff.dot(x_diff);
  const Scalar sol_lam2 = -x2_c2_diff.dot(x_diff);

  const Vector12s theta_dot = (Vector12s() << v1, v2, w1, w2).finished();

  const Scalar Ldot = x_diff.dot(v1 - v2) - x_diff_cross_x1_c1_diff.dot(w1) +
                      x_diff_cross_x2_c2_diff.dot(w2);

  const Scalar distance_inv = 1.0 / d->distance;
  const Scalar dist_dot = Ldot * distance_inv;

  // Precompute components of the matrix that are repeating
  const Matrix3s sol_lam1_A1 = sol_lam1 * A1;
  const Matrix3s sol_lam2_A2 = sol_lam2 * A2;
  const Vector3s A1_x1_c1_diff = A1 * x1_c1_diff;
  const Vector3s A2_x2_c2_diff = A2 * x2_c2_diff;
  const Matrix3s x1_c1_diff_skew = pinocchio::skew(x1_c1_diff);
  const Matrix3s x2_c2_diff_skew = pinocchio::skew(x2_c2_diff);
  // Fill Lyy matrix only with the blocks that are changing
  d->Lyy.template topLeftCorner<3, 3>().noalias() =
      -Matrix3s::Identity(3, 3) + sol_lam1_A1;
  d->Lyy.template block<3, 1>(0, 6).noalias() = A1_x1_c1_diff;
  d->Lyy.template block<3, 3>(3, 0).noalias() =
      Matrix3s::Identity(3, 3) + sol_lam2_A2;
  d->Lyy.template block<3, 1>(3, 7).noalias() = A2_x2_c2_diff;
  d->Lyy.template block<1, 3>(6, 2).noalias() = A1_x1_c1_diff;
  d->Lyy.template bottomRightCorner<1, 3>().noalias() = A2_x2_c2_diff;
  // Fill Lyc matrix only with the blocks that are changing
  d->Lyc.template topLeftCorner<3, 3>().noalias() = -sol_lam1_A1;
  d->Lyc.template block<3, 3>(3, 3).noalias() = -sol_lam2_A2;
  d->Lyc.template block<1, 3>(6, 0).noalias() = -A1_x1_c1_diff;
  d->Lyc.template bottomRightCorner<1, 3>().noalias() = -A2_x2_c2_diff;

  // Fill Lyr matrix only with the blocks that are changing
  d->Lyr.template topLeftCorner<3, 3>().noalias() =
      sol_lam1 * (A1 * x1_c1_diff_skew - pinocchio::skew(A1_x1_c1_diff));
  d->Lyr.template block<3, 3>(3, 3).noalias() =
      sol_lam2 * (A2 * x2_c2_diff_skew - pinocchio::skew(A2_x2_c2_diff));
  d->Lyr.template block<1, 3>(6, 0).noalias() =
      x1_c1_diff.transpose() * A1 * x1_c1_diff_skew;
  d->Lyr.template bottomRightCorner<1, 3>().noalias() =
      x2_c2_diff.transpose() * A2 * x2_c2_diff_skew;

  const Matrix8s Lyy_inv = -1.0 * d->Lyy.inverse();
  const Matrix86s yc = Lyy_inv * d->Lyc;
  const Matrix86s yr = Lyy_inv * d->Lyr;

  const Matrix36s &xc = yc.template topRows<3>();
  const Matrix36s &xr = yr.template topRows<3>();

  const Matrix312s dx1 = (Matrix312s() << xc, xr).finished();
  const Matrix312s dx2 = (Matrix312s() << yc.template middleRows<3>(3),
                          yr.template middleRows<3>(3))
                             .finished();

  // Precompute difference of vectors
  const Matrix312s dx_diff = dx1 - dx2;

  const Vector12s dL_dtheta =
      (Vector12s() << x_diff, -x_diff, -x_diff_cross_x1_c1_diff,
       x_diff_cross_x2_c2_diff)
          .finished();

  const Matrix3s x_diff_skew = pinocchio::skew(x_diff);

  Matrix1212s ddL_dtheta2;
  ddL_dtheta2 << dx_diff, -dx_diff,
      -x_diff_skew * dx1 + x1_c1_diff_skew * dx_diff,
      x_diff_skew * dx2 - x2_c2_diff_skew * dx_diff;
  ddL_dtheta2.template block<3, 3>(7, 0) =
      ddL_dtheta2.template block<3, 3>(7, 0) + x_diff_skew;
  ddL_dtheta2.template block<3, 3>(6, 3) =
      ddL_dtheta2.template block<3, 3>(7, 3) - x_diff_skew;

  const Scalar distance_inv_pow = distance_inv * distance_inv;
  const Vector12s d_dist_dot_dtheta =
      ((theta_dot.transpose() * ddL_dtheta2) * distance_inv).transpose() -
      (dist_dot * distance_inv_pow * dL_dtheta);

  pinocchio::computeFrameJacobian(
      pin_model_, *d->pinocchio, d->q, geom_1.parentFrame,
      pinocchio::LOCAL_WORLD_ALIGNED, d->d_theta1_dq);
  pinocchio::computeFrameJacobian(
      pin_model_, *d->pinocchio, d->q, geom_2.parentFrame,
      pinocchio::LOCAL_WORLD_ALIGNED, d->d_theta2_dq);

  // Assign valued to preallocated d_theta_dq matrix
  d->d_theta_dq.template topRows<3>().noalias() =
      d->d_theta1_dq.template topRows<3>();
  d->d_theta_dq.template middleRows<3>(3).noalias() =
      d->d_theta2_dq.template topRows<3>();
  d->d_theta_dq.template middleRows<3>(6).noalias() =
      d->d_theta1_dq.template bottomRows<3>();
  d->d_theta_dq.template bottomRows<3>().noalias() =
      d->d_theta2_dq.template bottomRows<3>();

  d->d_theta_dot_dq = pinocchio::computeJointJacobiansTimeVariation(
      pin_model_, *d->pinocchio, d->q, d->v);

  d->dJ.template topRows<3>() = d->d_theta_dot_dq.template topRows<3>();
  d->dJ.template middleRows<3>(6) = d->d_theta_dot_dq.template bottomRows<3>();

  const Vector12s d_dist_dot_dtheta_dot = distance_inv * dL_dtheta;

  d->d_dist_dot_dq.noalias() = d_dist_dot_dtheta.transpose() * d->d_theta_dq +
                               d_dist_dot_dtheta_dot.transpose() * d->dJ;

  // Assume memory is allocated
  d->ddistdot_dq_val.topRows(pin_model_.nq).noalias() = d->d_dist_dot_dq;
  d->ddistdot_dq_val.bottomRows(pin_model_.nq).noalias() =
      d_dist_dot_dtheta_dot.transpose() * d->d_theta_dq;

  // Transport the jacobian of frame 1 into the jacobian associated to x1
  // TODO I have no idea which one is cdata and rdata in here
  const Vector3s &p1 = d->pinocchio->oMf[geom_1.parentFrame].translation();
  d->f1Mp1.translation(x1 - p1);
  d->jacobian1 = d->f1Mp1.toActionMatrixInverse() * d->d_theta1_dq;

  const Vector3s &p2 = d->pinocchio->oMf[geom_2.parentFrame].translation();
  d->f2Mp2.translation(x2 - p2);
  d->jacobian2 = d->f2Mp2.toActionMatrixInverse() * d->d_theta2_dq;

  d->J =
      distance_inv * x_diff.transpose() *
      (d->jacobian1.template topRows<3>() - d->jacobian2.template topRows<3>());

  d->ddistdot_dq_val.topRows(nq).noalias() =
      d->ddistdot_dq_val.topRows(nq) - d->J;
  d->Rx.noalias() = d->ddistdot_dq_val * ksi_ / (di_ - ds_);
}
template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> >
ResidualModelVelocityAvoidanceTpl<Scalar>::createData(
    DataCollectorAbstract *const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this,
                                      data);
}

template <typename Scalar>
const pinocchio::GeometryModel &
ResidualModelVelocityAvoidanceTpl<Scalar>::get_geometry() const {
  return *geom_model_.get();
}

template <typename Scalar>
pinocchio::PairIndex ResidualModelVelocityAvoidanceTpl<Scalar>::get_pair_id()
    const {
  return pair_id_;
}

template <typename Scalar>
void ResidualModelVelocityAvoidanceTpl<Scalar>::set_pair_id(
    const pinocchio::PairIndex pair_id) {
  pair_id_ = pair_id;
}

template <typename Scalar>
Scalar ResidualModelVelocityAvoidanceTpl<Scalar>::get_di() const {
  return di_;
}

template <typename Scalar>
void ResidualModelVelocityAvoidanceTpl<Scalar>::set_di(const Scalar di) {
  di_ = di;
}

template <typename Scalar>
Scalar ResidualModelVelocityAvoidanceTpl<Scalar>::get_ds() const {
  return ds_;
}

template <typename Scalar>
void ResidualModelVelocityAvoidanceTpl<Scalar>::set_ds(const Scalar ds) {
  ds_ = ds;
}

template <typename Scalar>
Scalar ResidualModelVelocityAvoidanceTpl<Scalar>::get_ksi() const {
  return ksi_;
}

template <typename Scalar>
void ResidualModelVelocityAvoidanceTpl<Scalar>::set_ksi(const Scalar ksi) {
  ksi_ = ksi;
}

}  // namespace colmpc

#endif  // PINOCCHIO_WITH_HPP_FCL
#endif  // COLMPC_RESIDUAL_VELOCITY_AVOIDANCE_HXX_
