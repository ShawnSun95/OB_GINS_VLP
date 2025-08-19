/*
 * RSS correction methods have been added.
 * Created by Xiao Sun, Kun Zuo, 2025.06.
 */

#include "preintegration_base.h"

PreintegrationBase::PreintegrationBase(std::shared_ptr<IntegrationParameters> parameters, const IMU &imu0,
                                       IntegrationState state,std::shared_ptr<imu_vlp>vlp_1)
    : parameters_(std::move(parameters))
    , current_state_(std::move(state)) 
    , vlp_(std::move(vlp_1)){

    start_time_ = imu0.time;
    end_time_   = imu0.time;

    imu_buffer_.clear();
    imu_buffer_.push_back(imu0);

    gravity_ = Vector3d(0, 0, parameters_->gravity);

    dRSS_first.assign(vlp_->NLed, 0.0);
    dRSS_latter.assign(vlp_->NLed, 0.0);
}

void PreintegrationBase::integration(const IMU &imu_pre, const IMU &imu_cur) {
    // 区间时间累积
    double dt = imu_cur.dt;
    delta_time_ += dt;

    end_time_           = imu_cur.time;
    current_state_.time = imu_cur.time;

    // 连续状态积分, 先位置速度再姿态

    // 位置速度
    Vector3d dvfb = imu_cur.dvel + 0.5 * imu_cur.dtheta.cross(imu_cur.dvel) +
                    1.0 / 12.0 * (imu_pre.dtheta.cross(imu_cur.dvel) + imu_pre.dvel.cross(imu_cur.dtheta));
    Vector3d dvel = current_state_.q.toRotationMatrix() * dvfb + gravity_ * dt;

    current_state_.p += dt * current_state_.v + 0.5 * dt * dvel;
    current_state_.v += dvel;

    // 姿态
    Vector3d dtheta = imu_cur.dtheta + 1.0 / 12.0 * imu_pre.dtheta.cross(imu_cur.dtheta);
    current_state_.q *= Rotation::rotvec2quaternion(dtheta);
    current_state_.q.normalize();

    // 预积分
    dvel = delta_state_.q.toRotationMatrix() * dvfb;
    delta_state_.p += dt * delta_state_.v + 0.5 * dt * dvel;
    delta_state_.v += dvel;

    // 姿态
    delta_state_.q *= Rotation::rotvec2quaternion(dtheta);
    delta_state_.q.normalize();

    //将vlp数据传递进入函数
    std::vector<double>LED=vlp_->LED;
    int Nled=vlp_->NLed;
    int T=vlp_->windows;
    std::vector<double>A=vlp_->A;
    std::vector<double>M=vlp_->M;
    int hz=vlp_->hz;

    double ti=current_state_.time-vlp_->start_time;
    double tk=floor(ti)+T/2.0;

    for (int i = 0; i < Nled; i++){ 
    //计算每一时刻改正量
        Vector3d unit(0, 0, -1);
        Vector3d n_PD = current_state_.q.toRotationMatrix() * unit;
        Vector3d n_LED{0, 0, -1};
        
        Vector3d D{LED[i*3+1]-current_state_.p(0),LED[i*3+0]-current_state_.p(1),-LED[i*3+2]-current_state_.p(2)};
        
        // Vector3d coef1=-D.cross(n_PD)/D.dot(n_PD);
        // Vector3d coef2 = (-n_PD / n_PD.dot(D)).eval()
        //         - (M[i] * n_LED / n_LED.dot(D)).eval()
        //         + ((3 + M[i]) / D.squaredNorm()) * D;

        Vector3d coef3=-A[i]*D.cross(n_PD)*pow(D.dot(n_LED), M[i])/pow(D.norm(), 3+M[i]);
        Vector3d coef4=-A[i]*n_PD*pow(D.dot(n_LED), M[i])/pow(D.norm(), 3+M[i])
                -A[i]*M[i]*n_LED*D.dot(n_PD)*pow(D.dot(n_LED), M[i]-1)/pow(D.norm(), 3+M[i])
                +A[i]*(3+M[i])*D.dot(n_PD)*pow(D.dot(n_LED), M[i])*D/pow(D.norm(), 5+M[i]);

        double dp1=0.0;double dp2=0.0;
        if(D.dot(n_PD)/D.norm()<0.01 || D.dot(n_LED)/D.norm()<0.01)
            continue;
        //判断时刻
        if(ti>=floor(ti) && ti<tk){
            dp1=(tk-ti)*coef3.dot(dtheta)/T;
            dp1=dp1+(tk-ti)*coef4.dot(current_state_.v)/hz/T;
            dRSS_first[i]+=dp1;
        } else {
            //时间段的后半段
            dp2=-(ti-tk)*coef3.dot(dtheta)/T;
            dp2=dp2-(ti-tk)*coef4.dot(current_state_.v)/hz/T;
            dRSS_latter[i]+=dp2;
        }
    }
}

void PreintegrationBase::addNewImu(const IMU &imu) {
    imu_buffer_.push_back(imu);
    integrationProcess(imu_buffer_.size() - 1);
}

void PreintegrationBase::reintegration(IntegrationState &state) {
    current_state_ = std::move(state);
    resetState(current_state_);

    for (size_t k = 1; k < imu_buffer_.size(); k++) {
        integrationProcess(k);
    }
}

IMU PreintegrationBase::compensationBias(const IMU &imu) const {
    IMU imu_calib = imu;
    imu_calib.dtheta -= imu_calib.dt * delta_state_.bg;
    imu_calib.dvel -= imu_calib.dt * delta_state_.ba;

    return imu_calib;
}

IMU PreintegrationBase::compensationScale(const IMU &imu) const {
    IMU imu_calib = imu;

    for (int k = 0; k < 3; k++) {
        imu_calib.dtheta[k] *= (1.0 - delta_state_.sg[k]);
        imu_calib.dvel[k] *= (1.0 - delta_state_.sa[k]);
    }
    return imu_calib;
}

void PreintegrationBase::stateToData(const IntegrationState &state, IntegrationStateData &data) {
    data.time = state.time;

    memcpy(data.pose, state.p.data(), sizeof(double) * 3);
    memcpy(data.pose + 3, state.q.coeffs().data(), sizeof(double) * 4);

    memcpy(data.mix, state.v.data(), sizeof(double) * 3);
    memcpy(data.mix + 3, state.bg.data(), sizeof(double) * 3);
    memcpy(data.mix + 6, state.ba.data(), sizeof(double) * 3);
}

void PreintegrationBase::stateFromData(const IntegrationStateData &data, IntegrationState &state) {
    state.time = data.time;

    memcpy(state.p.data(), data.pose, sizeof(double) * 3);
    memcpy(state.q.coeffs().data(), data.pose + 3, sizeof(double) * 4);
    state.q.normalize();

    memcpy(state.v.data(), data.mix, sizeof(double) * 3);
    memcpy(state.bg.data(), data.mix + 3, sizeof(double) * 3);
    memcpy(state.ba.data(), data.mix + 6, sizeof(double) * 3);
}