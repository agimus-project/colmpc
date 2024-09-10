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
  const Vector3s Lr1.noalias() = c1.transpose() * pinocchio::skew(Lc) +
                       x2.transpose() * pinocchio::skew(x1);

  // Negate Lc inverting order of substraction to x1 - x2
  Lc.noalias() = -1.0 * Lc;
  const Vector3s Lr2.noalias() = c2.transpose() * pinocchio::skew(Lc) +
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
  const DiagonalMatrix3s D1 = static_cast<hppfcl::Elipsoid>(geom_1.geometry).radii.asDiagonal();
  const DiagonalMatrix3s D2 = static_cast<hppfcl::Elipsoid>(geom_2.geometry).radii.asDiagonal();
  // Store rotations of geometries in matrices
  const Matrix3s &R1 = geom_1.rotation();
  const Matrix3s &R2 = geom_2.rotation();
  
  const Matrix3s A1.noalias() = R1 * D1 * R1.transpose();
  const Matrix3s A2.noalias() = R1 * D1 * R1.transpose();

  // Precompute differences as they are often used;
  const Vector3s x_diff.noalias() = x1 - x2;
  const Vector3s x1_c1_diff.noalias() = x1 - c1;
  const Vector3s x2_c2_diff.noalias() = c2 - c2;
  // Precompute cross products as they are used many times
  const Vector3s x_diff_cross_x1_c1_diff.noalias() = x_diff.cross(x1_c1_diff);
  const Vector3s x_diff_cross_x2_c2_diff.noalias() = x_diff.cross(x2_c2_diff);

  const Scalar sol_lam1 = -x1_c1_diff.dot(x_diff);
  const Scalar sol_lam2 = -x2_c2_diff.dot(x_diff);

  const Vector12s theta_dot(v1, v2, w1, w2);

  const Scalar Ldot = 
      x_diff.dot(v1 - v2)
      - x_diff_cross_x1_c1_diff.dot(w1)
      + x_diff_cross_x2_c2_diff.dot(w2);

  const Scalar distance_inv = 1.0 / distance;
  const Scalar dist_dot = Ldot * distance_inv;


  // const Matrix8s Lyy = np.r_[
  //     np.c_[np.eye(3) + sol_lam1 * A1, -np.eye(3), A1 @ (x1_c1_diff), np.zeros(3)],
  //     np.c_[-np.eye(3), np.eye(3) + sol_lam2 * A2, np.zeros(3), A2 @ (x2_c2_diff)],
  //     [np.r_[A1 @ (x1_c1_diff), np.zeros(3), np.zeros(2)]],
  //     [np.r_[np.zeros(3), A2 @ (x2_c2_diff), np.zeros(2)]],
  // ]
  // const Matrix86s Lyc = np.r_[
  //     np.c_[-sol_lam1 * A1, np.zeros([3, 3])],
  //     np.c_[np.zeros([3, 3]), -sol_lam2 * A2],
  //     [np.r_[-A1 @ (x1_c1_diff), np.zeros(3)]],
  //     [np.r_[np.zeros(3), -A2 @ (x2_c2_diff)]],
  // ]
  // const Matrix86s Lyr = np.r_[
  //     np.c_[
  //         sol_lam1 * (A1 @ pin.skew(x1_c1_diff) - pin.skew(A1 @ (x1_c1_diff))),
  //         np.zeros([3, 3]),
  //     ],
  //     np.c_[
  //         np.zeros([3, 3]),
  //         sol_lam2 * (A2 @ pin.skew(x2_c2_diff) - pin.skew(A2 @ (x2_c2_diff))),
  //     ],
  //     [np.r_[(x1_c1_diff) @ A1 @ pin.skew(x1_c1_diff), np.zeros(3)]],
  //     [np.r_[np.zeros(3), (x2_c2_diff) @ A2 @ pin.skew(x2_c2_diff)]],
  // ]

  // TODO find faster inverse
  const Matrix8s Lyy_inv.noalias() = -1.0 * Lyy.inverse();
  const Matrix86s yc.noalias() = Lyy_inv * Lyc;
  const Matrix86s yr.noalias() = Lyy_inv * Lyr;

  const Matrix36s &xc = yc.template .<3>topRows();
  const Matrix36s &xr = yr.template .<3>topRows();

  const Matrix312s dx1(xc, xr);
  const Matrix312s dx2(yc.template .<3, 6>middleRows(), yr.template .<3, 6>middleRows() );
  // Precompute difference of vectors
  const Matrix312s dx_diff.noalias() = dx1 - dx2;

  const Vector12s dL_dtheta(x_diff, -x_diff, -x_diff_cross_x1_c1_diff, x_diff_cross_x2_c2_diff);


  // const Matrix1212s ddL_dtheta2 = (
  //     np.r_[
  //         dx_diff,
  //         -dx1 + dx2,
  //         -pin.skew(x_diff) @ dx1 + pin.skew(x1_c1_diff) @ (dx_diff),
  //         pin.skew(x_diff) @ dx2 - pin.skew(x2_c2_diff) @ (dx_diff),
  //     ]
  //     + np.r_[
  //         np.zeros([6, 12]),
  //         np.c_[pin.skew(x_diff), np.zeros([3, 9])],
  //         np.c_[np.zeros([3, 3]), -pin.skew(x_diff), np.zeros([3, 6])],
  //     ]
  // )

  const Scalara distance_inv_pow = distance_inv * distance_inv;
  const Vector12s d_dist_dot_dtheta.noalias() = ((theta_dot.transpose() * ddL_dtheta2) * distance_inv) - (dist_dot * distance_inv_pow * dL_dtheta);

  pinocchio::computeFrameJacobian(pin_model_, *d->pinocchio, d->q, geom_1.parentFrame, pinocchio::LOCAL_WORLD_ALIGNED, d->d_theta1_dq);
  pinocchio::computeFrameJacobian(pin_model_, *d->pinocchio, d->q, geom_2.parentFrame, pinocchio::LOCAL_WORLD_ALIGNED, d->d_theta2_dq);

  
  const Matrix12xLike d_theta_dq(
    d_theta1_dq.template .topRows<3>(),
    d_theta2_dq.template .topRows<3>(),
    d_theta1_dq.template .bottomRows<3>(),
    d_theta2_dq.template .bottomRows<3>(),
  );
  
  pinocchio::computeJointJacobiansTimeVariation(pin_model_, *d->pinocchio, d->q, d->v, d->d_theta_dot_dq);

  dJ.setZero() // TODO move to initialization
  dJ.template .topRows<3>() = d_theta_dot_dq.template .topRows<3>();
  dJ.template .block<3, 6>() = d_theta_dot_dq.template .topRows<3>();

  const Vector12s d_dist_dot_dtheta_dot.noalias() = distance_inv * dL_dtheta;

  d->d_dist_dot_dq.noalias() = d_dist_dot_dtheta * d_theta_dq + d_dist_dot_dtheta_dot * dJ;

  // Assume memory is allocated
  d->ddistdot_dq_val.topRows(pin_model_.nq).noalias() = d_dist_dot_dq; 
  d->ddistdot_dq_val.bottomRows(pin_model_.nq).noalias() = d_dist_dot_dtheta_dot * d_theta_dq;
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
