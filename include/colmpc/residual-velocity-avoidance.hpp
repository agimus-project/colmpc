///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2024, CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef COLMPC_RESIDUAL_VELOCITY_AVOIDANCE_HPP_
#define COLMPC_RESIDUAL_VELOCITY_AVOIDANCE_HPP_

#include "colmpc/fwd.hpp"
// include fwd first

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/distance.h>

#include <Eigen/Core>
#include <crocoddyl/core/residual-base.hpp>
#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/fcl.hpp>

#include "crocoddyl/core/utils/exception.hpp"

namespace colmpc {
using namespace crocoddyl;

template <typename _Scalar>
struct ResidualModelVelocityAvoidanceTpl
    : public ResidualModelAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataVelocityAvoidanceTpl<Scalar> Data;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef pinocchio::GeometryModel GeometryModel;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::Matrix6xs Matrix6xs;
  typedef typename MathBase::Vector3s Vector3s;
  /**
   * @brief Initialize the pair collision residual model
   *
   * @param[in] state       State of the multibody system
   * @param[in] nu          Dimension of the control vector
   * @param[in] geom_model  Pinocchio geometry model containing the collision
   * pair
   * @param[in] pair_id     Index of the collision pair in the geometry model
   * @param[in] di          Distance at which the robot starts to slow down
   * @param[in] ds          Security distance
   * @param[in] ksi         Convergence speed coefficient
   */

  ResidualModelVelocityAvoidanceTpl(boost::shared_ptr<StateMultibody> state,
                                    const std::size_t nu,
                                    boost::shared_ptr<GeometryModel> geom_model,
                                    const pinocchio::PairIndex pair_id,
                                    const Scalar di = 1.0e-2,
                                    const Scalar ds = 1.0e-5,
                                    const Scalar ksi = 1.0e-2);
  virtual ~ResidualModelVelocityAvoidanceTpl();

  /**
   * @brief Compute the pair collision residual
   *
   * @param[in] data  Pair collision residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract> &data,
                    const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u);

  /**
   * @brief Compute the derivatives of the pair collision residual
   *
   * @param[in] data  Pair collision residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract> &data,
                        const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u);

  virtual boost::shared_ptr<ResidualDataAbstract> createData(
      DataCollectorAbstract *const data);

  /**
   * @brief Return the Pinocchio geometry model
   */
  const GeometryModel &get_geometry() const;

  /**
   * @brief Return the reference collision pair id
   */
  pinocchio::PairIndex get_pair_id() const;

  /**
   * @brief Modify the reference collision pair id
   */
  void set_pair_id(const pinocchio::PairIndex pair_id);

  /**
   * @brief Return the distance at which the robot starts to slow down
   */
  Scalar get_di() const;

  /**
   * @brief Modify the distance at which the robot starts to slow down
   */
  void set_di(const Scalar di);

  /**
   * @brief Return the security distance
   */
  Scalar get_ds() const;

  /**
   * @brief Modify the security distance
   */
  void set_ds(const Scalar ds);

  /**
   * @brief Return the convergence speed coefficient
   */
  Scalar get_ksi() const;

  /**
   * @brief Modify the convergence speed coefficient
   */
  void set_ksi(const Scalar ksi);

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  typename StateMultibody::PinocchioModel
      pin_model_;  //!< Pinocchio model used for internal computations
  boost::shared_ptr<GeometryModel>
      geom_model_;  //!< Pinocchio geometry model containing collision pair
  pinocchio::PairIndex
      pair_id_;  //!< Index of the collision pair in geometry model

  Scalar di_;   //!< Distance at which the robot starts to slow down
  Scalar ds_;   //!< Security distance
  Scalar ksi_;  //!< Convergence speed coefficient
};

template <typename _Scalar>
struct ResidualDataVelocityAvoidanceTpl
    : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;

  typedef pinocchio::DataTpl<Scalar> PinocchioData;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Matrix6xs Matrix6xs;
  typedef typename MathBase::Vector3s Vector3s;

  template <template <typename Scalar> class Model>
  ResidualDataVelocityAvoidanceTpl(Model<Scalar> *const model,
                                   DataCollectorAbstract *const data)
      : Base(model, data),
        geometry(pinocchio::GeometryData(model->get_geometry())),
        req(),
        res() {
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar> *d =
        dynamic_cast<DataCollectorMultibodyTpl<Scalar> *>(shared);
    if (d == NULL) {
      throw_pretty(
          "Invalid argument: the shared data should be derived from "
          "DataCollectorActMultibodyTpl");
    }

    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
  }
  pinocchio::GeometryData geometry;       //!< Pinocchio geometry data
  pinocchio::DataTpl<Scalar> *pinocchio;  //!< Pinocchio data

  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;

  hpp::fcl::DistanceRequest
      req;  //!< Distance Request from hppfcl,
            //!< used to compute the distance between shapes
  hpp::fcl::DistanceResult res;  //!< Distance Result from hppfcl

  pinocchio::SE3 oMg_id_1;
  pinocchio::SE3 oMg_id_2;
};

}  // namespace colmpc

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "colmpc/residual-velocity-avoidance.hxx"
#endif  // COLMPC_RESIDUAL_VELOCITY_AVOIDANCE_HPP_
