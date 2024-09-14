///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2024, CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef COLMPC_RESIDUAL_VELOCITY_AVOIDANCE_HXX_
#define COLMPC_RESIDUAL_VELOCITY_AVOIDANCE_HXX_
#ifdef PINOCCHIO_WITH_HPP_FCL

#include "colmpc/residual-velocity-avoidance.hpp"

namespace colmpc {
using namespace crocoddyl;

template <typename Scalar>
ResidualModelVelocityAvoidanceTpl<Scalar>::ResidualModelVelocityAvoidanceTpl(
    boost::shared_ptr<StateMultibody> state,
    boost::shared_ptr<GeometryModel> geom_model,
    const pinocchio::PairIndex pair_id, const Scalar di, const Scalar ds,
    const Scalar ksi)
    : Base(state, 1, true, true, true),
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

  pinocchio::forwardKinematics(pin_model_, *d->pinocchio, d->q, d->v);
  std::cout << "d->q" << d->q << std::endl;
  std::cout << "d->v" << d->v << std::endl;
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
  std::cout << "geom_1.placement.tr" << geom_1.placement.translation()
            << std::endl;
  std::cout << "geom_2.placement.tr" << geom_2.placement.translation()
            << std::endl;

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

  std::cout << "&v1" << v1 << std::endl;
  std::cout << "&v2" << v2 << std::endl;
  std::cout << "&w1" << w1 << std::endl;
  std::cout << "&w2" << w2 << std::endl;
  std::cout << "toto" << std::endl << std::flush;
  // Precompute differences as they are often used;
  d->x_diff = x1 - x2;
  d->x1_c1_diff = x1 - c1;
  d->x2_c2_diff = x2 - c2;
  // Precompute cross products as they are used many times
  d->x_diff_cross_x1_c1_diff.noalias() = d->x_diff.cross(d->x1_c1_diff);
  d->x_diff_cross_x2_c2_diff.noalias() = d->x_diff.cross(d->x2_c2_diff);

  d->Ldot = d->x_diff.dot(v1 - v2) - d->x_diff_cross_x1_c1_diff.dot(w1) +
            d->x_diff_cross_x2_c2_diff.dot(w2);
  data->r[0] =
      (d->Ldot / d->distance) + ksi_ * (d->distance - ds_) / (di_ - ds_);
}

template <typename Scalar>
void ResidualModelVelocityAvoidanceTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ResidualDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &x, const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());

  const std::size_t nq = state_->get_nq();
  const std::size_t nv = state_->get_nv();
  d->q = x.head(nq);
  d->v = x.tail(nv);

  // Create labels for geometries
  const auto &cp = geom_model_->collisionPairs[pair_id_];
  const auto &geom_1 = geom_model_->geometryObjects[cp.first];
  const auto &geom_2 = geom_model_->geometryObjects[cp.second];
  // Crate labels for points
  const Vector3s &x1 = d->res.nearest_points[0];
  const Vector3s &x2 = d->res.nearest_points[1];
  // Create labels for velocities
  const Vector3s &v1 = d->m1.linear();
  const Vector3s &v2 = d->m2.linear();
  const Vector3s &w1 = d->m1.angular();
  const Vector3s &w2 = d->m2.angular();
  // Convert sphere representation to a diagonal matrix

  // For geom_1
  // For geom_1
  const double radius1 =
      std::static_pointer_cast<hpp::fcl::Sphere>(geom_1.geometry)->radius;
  DiagonalMatrix3s D1;
  D1.diagonal() << 1.0 / radius1, 1.0 / radius1, 1.0 / radius1;

  // For geom_2
  const double radius2 =
      std::static_pointer_cast<hpp::fcl::Sphere>(geom_2.geometry)->radius;
  DiagonalMatrix3s D2;
  D2.diagonal() << 1.0 / radius2, 1.0 / radius2, 1.0 / radius2;

  // const DiagonalMatrix3s D1 =
  //     1/ std::static_pointer_cast<hpp::fcl::Sphere>(geom_1.geometry)
  //       ->radius
  //         .asDiagonal();
  // const DiagonalMatrix3s D2 =
  //     1 / std::static_pointer_cast<hpp::fcl::Sphere>(geom_2.geometry)
  //       ->radius
  //         .asDiagonal();

  // Store rotations of geometries in matrices
  const Matrix3s &R1 = d->oMg_id_1.rotation();
  const Matrix3s &R2 = d->oMg_id_2.rotation();

  const Matrix3s A1 = R1 * D1 * D1 * R1.transpose();
  const Matrix3s A2 = R2 * D2 * D2 * R2.transpose();

  const Scalar sol_lam1 = -d->x1_c1_diff.dot(d->x_diff);
  const Scalar sol_lam2 = d->x2_c2_diff.dot(d->x_diff);

  // Precompute inverse od distance for faster operations
  const Scalar distance_inv = 1.0 / d->distance;
  const Scalar dist_dot = d->Ldot * distance_inv;

  // Precompute components of the matrix that are repeating
  const Vector3s A1_x1_c1_diff = A1 * d->x1_c1_diff;
  const Vector3s A2_x2_c2_diff = A2 * d->x2_c2_diff;
  const Matrix3s sol_lam1_A1 = sol_lam1 * A1;
  const Matrix3s sol_lam2_A2 = sol_lam2 * A2;
  const Matrix3s x1_c1_diff_skew = pinocchio::skew(d->x1_c1_diff);
  const Matrix3s x2_c2_diff_skew = pinocchio::skew(d->x2_c2_diff);

  // Fill Lyc matrix only with the blocks that are changing
  d->Lyc.template topLeftCorner<3, 3>() = -sol_lam1_A1;
  d->Lyc.template block<3, 3>(3, 3) = -sol_lam2_A2;
  d->Lyc.template block<1, 3>(6, 0) = -A1_x1_c1_diff;
  d->Lyc.template bottomRightCorner<1, 3>() = -A2_x2_c2_diff;

  // Fill Lyr matrix only with the blocks that are changing
  d->Lyr.template topLeftCorner<3, 3>().noalias() =
      sol_lam1 * (A1 * x1_c1_diff_skew - pinocchio::skew(A1_x1_c1_diff));
  d->Lyr.template block<3, 3>(3, 3).noalias() =
      sol_lam2 * (A2 * x2_c2_diff_skew - pinocchio::skew(A2_x2_c2_diff));
  d->Lyr.template block<1, 3>(6, 0).noalias() =
      d->x1_c1_diff.transpose() * A1 * x1_c1_diff_skew;
  d->Lyr.template bottomRightCorner<1, 3>().noalias() =
      d->x2_c2_diff.transpose() * A2 * x2_c2_diff_skew;

  // Optimized Lyy explicit inversion
  // Inversion is done as inversion of a block matrix in a form
  // |  M   b.T |-1   | M^-1 - M^-1 b N b^T M^-1   N M^-1 b |
  // |          |   = |                                     |
  // |  b    0  |     |      (N M^-1 b)^T             -N    |
  // Where N = (b^T M^-1 b)^-1

  // Invert upper left block of Lyy matrix
  //
  //     | M1  -I |-1   |    (M1 - M2^-1)^-1     (M1 - M2^-1)^-1 M2^-1 |
  // M = |        |   = |                                              |
  //     | -I  M2 |     | (M2 - M1^-1)^-1 M1^-1     (M2 - M1^-1)^-1    |
  //
  const Matrix3s M1 = Matrix3s::Identity(3, 3) + sol_lam1_A1;
  const Matrix3s M2 = Matrix3s::Identity(3, 3) + sol_lam2_A2;
  const Matrix3s M1_inv = M1.inverse();
  const Matrix3s M2_inv = M2.inverse();
  const Matrix3s M1_M2_inv_inv = (M1 - M2_inv).inverse();
  const Matrix3s M2_M1_inv_inv = (M2 - M1_inv).inverse();
  // Compute upper left, upper right, lower left and lower right block of M^-1
  const Matrix3s &M_up_l = M1_M2_inv_inv;
  const Matrix3s M_up_r = M1_M2_inv_inv * M2_inv;
  const Matrix3s M_lw_l = M2_M1_inv_inv * M1_inv;
  const Matrix3s &M_lw_r = M2_M1_inv_inv;
  // Compose final M inverse matrix
  const Matrix6s M_inv =
      (Matrix6s() << M_up_l, M_up_r, M_lw_l, M_lw_r).finished();
  // Compose final inverse of Lyy.
  // b vector is block sparse, hence full matrix dot product can be composed of
  // series of smaller vector X matrix X vector products.
  const Vector3s &b1 = A1_x1_c1_diff;  // First non zero vector of b
  const Vector3s &b2 = A2_x2_c2_diff;  // Second non zero vector of b
  // Precompute M^-1 b
  const Matrix62s M_inv_b =
      (Matrix62s() << M_up_l * b1, M_up_r * b2, M_lw_l * b1, M_lw_r * b2)
          .finished();
  // Use only non zero components of the b vector to finish computing N
  const Matrix2s N =
      (Matrix2s() << b1.transpose() * M_inv_b.template topRows<3>(),
       b2.transpose() * M_inv_b.template bottomRows<3>())
          .finished()
          .inverse();
  const Matrix62s M_inv_b_N = M_inv_b * N;
  const Matrix8s Lyy_inv =
      (Matrix8s() << M_inv - M_inv_b_N * M_inv_b.transpose(), M_inv_b_N,
       M_inv_b_N.transpose(), -N)
          .finished();

  const Matrix812s Lyth =
      (Matrix812s() << d->Lyc.template leftCols<3>(),
       d->Lyr.template leftCols<3>(), d->Lyc.template rightCols<3>(),
       d->Lyr.template rightCols<3>())
          .finished();

  const Matrix812s yth = -Lyy_inv * Lyth;

  const Matrix312s &dx1 = yth.template topRows<3>();
  const Matrix312s &dx2 = yth.template middleRows<3>(3);

  // Precompute difference of vectors
  const Matrix312s dx_diff = dx1 - dx2;

  const Vector12s dL_dtheta =
      (Vector12s() << d->x_diff, -d->x_diff_cross_x1_c1_diff, -d->x_diff,
       d->x_diff_cross_x2_c2_diff)
          .finished();

  const Matrix3s x_diff_skew = pinocchio::skew(d->x_diff);

  Matrix12s ddL_dtheta2;
  // Initialize matrix
  ddL_dtheta2 << dx_diff, -x_diff_skew * dx1 + x1_c1_diff_skew * dx_diff,
      -dx_diff, x_diff_skew * dx2 - x2_c2_diff_skew * dx_diff;
  // Update only certain blocks of the matrix
  ddL_dtheta2.template block<3, 3>(3, 0) =
      ddL_dtheta2.template block<3, 3>(3, 0) + x_diff_skew;
  ddL_dtheta2.template block<3, 3>(9, 6) =
      ddL_dtheta2.template block<3, 3>(9, 6) - x_diff_skew;

  // Compute (1 / distance)^2
  const Scalar distance_inv_pow = distance_inv * distance_inv;
  const Vector12s theta_dot = (Vector12s() << v1, w1, v2, w2).finished();
  const Vector12s d_dist_dot_dtheta =
      ((theta_dot.transpose() * ddL_dtheta2) * distance_inv).transpose() -
      (dist_dot * distance_inv_pow * dL_dtheta);

  pinocchio::computeJointJacobians(pin_model_, *d->pinocchio, d->q);
  pinocchio::computeFrameJacobian(pin_model_, *d->pinocchio, d->q,
                                  geom_1.parentFrame,
                                  pinocchio::LOCAL_WORLD_ALIGNED, d->J1);
  pinocchio::computeFrameJacobian(pin_model_, *d->pinocchio, d->q,
                                  geom_2.parentFrame,
                                  pinocchio::LOCAL_WORLD_ALIGNED, d->J2);

  d->d_theta_dq << d->J1, d->J2;

  pinocchio::computeForwardKinematicsDerivatives(pin_model_, *d->pinocchio,
                                                 d->q, d->v, d->__a);
  pinocchio::getFrameVelocityDerivatives(pin_model_, *d->pinocchio,
                                         geom_1.parentFrame, pinocchio::LOCAL,
                                         d->in1_dnu1_dq, d->in1_dnu1_dqdot);
  pinocchio::getFrameVelocityDerivatives(pin_model_, *d->pinocchio,
                                         geom_2.parentFrame, pinocchio::LOCAL,
                                         d->in2_dnu2_dq, d->in2_dnu2_dqdot);

  // Rewrite data to d_theta_dot_dq vector. Only rows 0:3 and 6:9 are non zero
  d->d_theta_dot_dq << R1 * d->in1_dnu1_dq.template topRows<3>() -
                           pinocchio::skew(v1) * R1 *
                               d->in1_dnu1_dqdot.template bottomRows<3>(),
      R1 * d->in1_dnu1_dq.template bottomRows<3>(),
      R2 * d->in2_dnu2_dq.template topRows<3>() -
          pinocchio::skew(v2) * R2 * d->in2_dnu2_dqdot.template bottomRows<3>(),
      R2 * d->in2_dnu2_dq.template bottomRows<3>();

  const Vector12s d_dist_dot_dtheta_dot = dL_dtheta * distance_inv;
  d->d_dist_dot_dq.noalias() =
      d_dist_dot_dtheta.transpose() * d->d_theta_dq +
      d_dist_dot_dtheta_dot.transpose() * d->d_theta_dot_dq;

  // Transport the jacobian of frame 1 into the jacobian associated to x1
  const Vector3s &p1 = d->pinocchio->oMf[geom_1.parentFrame].translation();
  d->f1Mp1.translation(x1 - p1);
  d->jacobian1.noalias() = d->f1Mp1.toActionMatrixInverse() * d->J1;

  const Vector3s &p2 = d->pinocchio->oMf[geom_2.parentFrame].translation();
  d->f2Mp2.translation(x2 - p2);
  d->jacobian2.noalias() = d->f2Mp2.toActionMatrixInverse() * d->J2;

  d->J.noalias() =
      distance_inv * d->x_diff.transpose() *
      (d->jacobian1.template topRows<3>() - d->jacobian2.template topRows<3>());

  d->ddistdot_dq_val.topRows(nq) =
      d->d_dist_dot_dq - (d->J * ksi_ / (di_ - ds_));
  // Compute and store d_dist_dot_dqdot
  const Matrix12xLike &dtheta_dot_dqdot = d->d_theta_dq;
  d->ddistdot_dq_val.bottomRows(nv).noalias() =
      d_dist_dot_dtheta_dot.transpose() * dtheta_dot_dqdot;

  data->Rx = d->ddistdot_dq_val.transpose().eval();
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
