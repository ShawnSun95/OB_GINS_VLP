/*
 * This factor is for tightly integrated VLP/INS systems.
 * Created by Xiao Sun.
 */

#ifndef VLP_FACTOR2_H
#define VLP_FACTOR2_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/earth.h"
#include "src/common/rotation.h"
#include "src/common/types.h"

const double clight = 299792458.0;

class VLPFactor2 : public ceres::SizedCostFunction<6, 7> {

public:
    explicit VLPFactor2(VLP vlp, Vector3d lever, int Nled_, vector<double> power, 
        vector<double> M_, vector<double> LED_)
        : vlp_(std::move(vlp))
        , lever_(std::move(lever)) {
            Nled = Nled_;
            a = new double[Nled];
            M = new double[Nled];
            LED = new double[Nled*3];
            for (int i = 0; i < Nled; i++){
                a[i] = power[i];
                M[i] = M_[i];
                for (int j = 0; j < 3; j++)
                    LED[i * 3 + j] = LED_[i * 3 + j];
            }
    }

    void updateVlpState(const VLP &vlp) {
        vlp_ = vlp;
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};
        
        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual.setZero();
        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian(jacobians[0]);
                jacobian.setZero();

                Vector3d unit(0, 0, -1);
                Vector3d n_PD = q.toRotationMatrix() * unit;
                Vector3d n_LED{0, 0, -1};

                for (int i = 0; i < Nled; i++){
                    double h = LED[i*3+2] + p(2);
                    double s = sqrt((LED[i*3+1]-p(0))*(LED[i*3+1]-p(0))+(LED[i*3+0]-p(1))*(LED[i*3+0]-p(1)));
                    Vector3d LOS{LED[i*3+1] - p(0), LED[i*3+0] - p(1), -h}; // NED system
                    double cos1 = LOS.dot(n_PD) / sqrt(h * h + s * s);
                    double cos2 = LOS.dot(n_LED) / sqrt(h * h + s * s);
                    double P = a[i] * cos1 * pow(cos2, M[i]) / (h * h + s * s);
                    residual(i, 0) = P - vlp_.RSS[i];
                    residual(i, 0) = residual(i, 0) / vlp_.RSS_std[i];

                    jacobian(i, 0) = P * (-n_PD[0] / LOS.dot(n_PD) + (3 + M[i])*(LED[i*3+1] - p(0))/(h * h + s * s));
                    jacobian(i, 1) = P * (-n_PD[1] / LOS.dot(n_PD) + (3 + M[i])*(LED[i*3+0] - p(1))/(h * h + s * s));
                    jacobian(i, 2) = P * (-n_PD[2] / LOS.dot(n_PD) -M[i]*n_LED[2]/n_LED.dot(LOS) + 
                        (3 + M[i])*(-LED[i*3+2] + p(2))/(h * h + s * s));

                    jacobian.block<1, 3>(i, 3) = -P * LOS.cross(n_PD) / LOS.dot(n_PD);
                    jacobian.block<1, 3>(i, 3) = jacobian.block<1, 3>(i, 3) * q.toRotationMatrix();
                    jacobian.block<1, 7>(i, 0) = jacobian.block<1, 7>(i, 0) / vlp_.RSS_std[i];
                }
            }
        }
        return true;
    }

private:
    VLP vlp_;
    Vector3d lever_;

    int Nled;
    double *a;
    double *M;
    double *LED;
};

#endif // VLP_FACTOR2_H
