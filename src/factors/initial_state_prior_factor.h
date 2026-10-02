#ifndef INITIAL_STATE_PRIOR_FACTOR_H
#define INITIAL_STATE_PRIOR_FACTOR_H

#include <ceres/ceres.h>
#include "src/common/rotation.h"

// Local attitude derivatives follow PoseParameterization's right increments.
// Only attitude and velocity are constrained; position and biases remain free.
class InitialStatePriorFactor : public ceres::CostFunction {
public:
    InitialStatePriorFactor(const Quaterniond &q, const Vector3d &velocity,
                            const Vector3d &attitude_std, const Vector3d &velocity_std,
                            int mix_size)
        : q_(q), velocity_(velocity), attitude_weight_(attitude_std.cwiseInverse()),
          velocity_weight_(velocity_std.cwiseInverse()), mix_size_(mix_size) {
        set_num_residuals(6);
        mutable_parameter_block_sizes()->push_back(7);
        mutable_parameter_block_sizes()->push_back(mix_size);
    }

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        const Eigen::Map<const Quaterniond> q(parameters[0] + 3);
        const Eigen::Map<const Vector3d> velocity(parameters[1]);
        Quaterniond error = q_.conjugate() * q;
        if (error.w() < 0) error.coeffs() *= -1;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual.head<3>() = attitude_weight_.asDiagonal() * (2 * error.vec());
        residual.tail<3>() = velocity_weight_.asDiagonal() * (velocity - velocity_);
        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobians[0]);
            j.setZero();
            j.block<3, 3>(0, 3) = attitude_weight_.asDiagonal() *
                (error.w() * Matrix3d::Identity() + Rotation::skewSymmetric(error.vec()));
        }
        if (jacobians && jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor>>
                j(jacobians[1], 6, mix_size_);
            j.setZero();
            j.block<3, 3>(3, 0) = velocity_weight_.asDiagonal();
        }
        return true;
    }

private:
    Quaterniond q_;
    Vector3d velocity_, attitude_weight_, velocity_weight_;
    int mix_size_;
};

#endif
