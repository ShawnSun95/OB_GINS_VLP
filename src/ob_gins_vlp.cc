/*
 * OB_GINS_VLP: An Optimization-Based VLP/INS Integrated Navigation System
 *
 * Copyright (C) 2024 Wuhan University
 *
 *     Author : xiao Sun
 *    Contact : xsun@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "src/common/earth.h"
#include "src/common/types.h"

#include "src/fileio/filesaver.h"
#include "src/fileio/vlpfileloader.h"
#include "src/fileio/imufileloader.h"

#include "src/factors/vlp_factor.h"
#include "src/factors/vlp_factor2.h"
#include "src/factors/NHC_factor.h"
#include "src/factors/marginalization_factor.h"
#include "src/factors/pose_parameterization.h"
#include "src/preintegration/imu_error_factor.h"
#include "src/preintegration/preintegration.h"
#include "src/preintegration/preintegration_factor.h"

#include <absl/time/clock.h>
#include <iomanip>
#include <yaml-cpp/yaml.h>

#define INTEGRATION_LENGTH 1.0
#define MINIMUM_INTERVAL 0.001

using ceres::CauchyLoss;

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

void writeNavResult(double time, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile);

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
        return -1;
    }

    auto ts = absl::Now();

    // 读取配置
    // load configuration
    YAML::Node config;
    std::vector<double> vec;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file" << std::endl;
        return -1;
    }

    // 时间信息
    // processing time
    int windows   = config["windows"].as<int>();
    int starttime = config["starttime"].as<int>();
    int endtime   = config["endtime"].as<int>();

    // 初始化信息
    // initialization
    vec = config["initcoordinate"].as<std::vector<double>>();
    Vector3d station_origin(vec[0]*D2R,vec[1]*D2R,vec[2]);
    vec = config["initpos"].as<std::vector<double>>();
    Vector3d initial_pos(vec[0],vec[1],vec[2]);
    vec = config["initvel"].as<std::vector<double>>();
    Vector3d initvel(vec.data());
    vec = config["initatt"].as<std::vector<double>>();
    Vector3d initatt(vec.data());
    initatt *= D2R;

    vec = config["initgb"].as<std::vector<double>>();
    Vector3d initbg(vec.data());
    initbg *= D2R / 3600.0;
    vec = config["initab"].as<std::vector<double>>();
    Vector3d initba(vec.data());
    initba *= 1.0e-5;

    // 数据文件
    // data file
    std::string vlppath   = config["vlpfile"].as<std::string>();
    std::string imupath    = config["imufile"].as<std::string>();
    std::string outputpath = config["outputpath"].as<std::string>();
    int imudatarate        = config["imudatarate"].as<int>();

    // integration scheme
    int scheme = 1;
    try {
        scheme = config["scheme"].as<int>();
    } catch (YAML::Exception &exception) {
        std::cout << "Default setting: loosely couple." << std::endl;
    }
    int imu_type = config["imu_type"].as<int>();

    bool INS_only = false;
    try {
        INS_only= config["INS_only"].as<bool>();
    } catch (YAML::Exception &exception) { }

    // 安装参数
    // installation parameters
    vec = config["antlever"].as<std::vector<double>>();
    Vector3d antlever(vec.data());
    vec = config["odolever"].as<std::vector<double>>();
    Vector3d odolever(vec.data());
    vec = config["bodyangle"].as<std::vector<double>>();
    Vector3d bodyangle(vec.data());
    bodyangle *= D2R;

    int Nled = 5;
    std::vector<double>vlp_power = config["emission_power"].as<std::vector<double>>();
    std::vector<double>M={1,1,1,1,1};
    std::vector<double>err={0.05,0.05,0.05};
    std::vector<double>LED={0.351,1.342,2.832,3.56,1.15,2.841,1.711,3.305,2.840,3.5,6.249,2.833,0.352,5.972,2.839};
    try {
        Nled = config["NLED"].as<int>();
    } catch (YAML::Exception &exception) { }
    try {
        M = config["lamber_coef"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) { }
    try {
        LED = config["LED"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) { }
    try {
        err = config["err"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) { }

    // IMU噪声参数
    // IMU noise parameters
    auto parameters          = std::make_shared<IntegrationParameters>();
    parameters->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
    parameters->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
    parameters->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
    parameters->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
    parameters->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;

    bool isuseodo       = config["odometer"]["isuseodo"].as<bool>();
    vec                 = config["odometer"]["std"].as<std::vector<double>>();
    parameters->odo_std = Vector3d(vec.data());
    parameters->odo_srw = config["odometer"]["srw"].as<double>() * 1e-6;
    parameters->lodo    = odolever;
    parameters->abv     = bodyangle;
    Matrix3d Cbv = Rotation::euler2matrix(bodyangle);
    initbg = Cbv * initbg;
    initba = Cbv *initba;

    VLPFileLoader vlpfile(vlppath, Nled, err);
    ImuFileLoader imufile(imupath, Cbv, imudatarate, imu_type);
    FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);
    FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.nav", 7, FileSaver::TEXT);
    if (!imufile.isOpen() || !navfile.isOpen() || !navfile.isOpen() || !errfile.isOpen()) {
        std::cout << "Failed to open data file" << std::endl;
        return -1;
    }

    // VLP仿真中断配置
    // VLP outage parameters
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime   = config["outagetime"].as<int>();
    int outagelen    = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();

    auto vlpthreshold = config["vlpthreshold"].as<double>();

    parameters->gravity     = Earth::gravity(station_origin);

    // 数据文件调整
    // data alignment
    IMU imu_cur, imu_pre;
    do {
        imu_pre = imu_cur;
        imu_cur = imufile.next(parameters->gravity);
    } while (imu_cur.time < starttime);

    VLP vlp;
    do {
        vlp = vlpfile.next();
    } while (vlp.time < starttime);

    // 初始位置, 求相对
    if(scheme == 2){
        vlp.xyz = initial_pos;
    }

    // 站心坐标系原点
    parameters->station = station_origin;

    std::vector<std::shared_ptr<PreintegrationBase>> preintegrationlist;
    std::vector<IntegrationState> statelist(windows + 1);
    std::vector<IntegrationStateData> statedatalist(windows + 1);
    std::vector<VLP> vlplist;
    std::vector<double> timelist;

    Preintegration::PreintegrationOptions preintegration_options = Preintegration::getOptions(isuseodo);

    // 初始状态
    // initialization
    IntegrationState state_curr = {
        .p    = initial_pos - Rotation::euler2quaternion(initatt) * antlever,
        .q    = Rotation::euler2quaternion(initatt),
        .v    = initvel,
        .bg   = initbg,
        .ba   = initba,
        .sodo = 0.0,
        .abv  = {bodyangle[1], bodyangle[2]},
    };
    std::cout << "Initilization at " << vlp.time << " s " << std::endl;

    statelist[0]     = state_curr;
    statedatalist[0] = Preintegration::stateToData(state_curr, preintegration_options);
    vlplist.push_back(vlp);

    double sow = round(vlp.time);
    timelist.push_back(sow);

    // 初始预积分
    // Initial preintegration
    preintegrationlist.emplace_back(
        Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));

    // 读取下一个整秒vlp
    vlp = vlpfile.next();

    // 边缘化信息
    std::shared_ptr<MarginalizationInfo> last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;

    // 下一个积分节点
    sow += INTEGRATION_LENGTH;

    // Pure INS mechanization
    while (INS_only) {
        if ((imu_cur.time > endtime) || imufile.isEof()) {
            break;
        }

        // 加入IMU数据
        // Add new imu data to preintegration
        preintegrationlist.back()->addNewImu(imu_cur);

        imu_pre = imu_cur;
        imu_cur = imufile.next(parameters->gravity);
        state_curr  = preintegrationlist.back()->currentState();
        writeNavResult(imu_cur.time-starttime, state_curr, navfile, errfile);
    }

    while (true) {
        if ((imu_cur.time > endtime) || imufile.isEof()) {
            break;
        }

        // 加入IMU数据
        // Add new imu data to preintegration
        preintegrationlist.back()->addNewImu(imu_cur);

        imu_pre = imu_cur;
        imu_cur = imufile.next(parameters->gravity);

        if (imu_cur.time > sow) {
            // 当前IMU数据时间等于vlp数据时间, 读取新的vlp
            // add vlp and read new vlp
            if (fabs(vlp.time - sow) < MINIMUM_INTERVAL) {
                vlplist.push_back(vlp);

                vlp = vlpfile.next();
                while ((vlp.std[0] > vlpthreshold) || (vlp.std[1] > vlpthreshold) ||
                    (vlp.std[2] > vlpthreshold)) {
                    vlp = vlpfile.next();
                }

                // 中断配置
                // do vlp outage
                if (isuseoutage) {
                    if (lround(vlp.time) == outagetime) {
                        std::cout << "vlp outage at " << outagetime << " s" << std::endl;
                        for (int k = 0; k < outagelen; k++) {
                            vlp = vlpfile.next();
                        }
                        outagetime += outageperiod;
                    }
                }

                if (vlpfile.isEOF()) {
                    vlp.time = 0;
                }
            }

            // IMU内插处理
            // IMU interpolation
            int isneed = isNeedInterpolation(imu_pre, imu_cur, sow);
            if (isneed == -1) {
            } else if (isneed == 1) {
                preintegrationlist.back()->addNewImu(imu_cur);

                imu_pre = imu_cur;
                imu_cur = imufile.next(parameters->gravity);
            } else if (isneed == 2) {
                imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
                preintegrationlist.back()->addNewImu(imu_pre);
            }

            // 下一个积分节点
            // next time node
            timelist.push_back(sow);
            sow += INTEGRATION_LENGTH;

            // 当前整秒状态加入到滑窗中
            //
            state_curr                               = preintegrationlist.back()->currentState();
            statelist[preintegrationlist.size()]     = state_curr;
            statedatalist[preintegrationlist.size()] = Preintegration::stateToData(state_curr, preintegration_options);

            // 构建优化问题
            // construct optimization problem
            {
                ceres::Solver solver;
                ceres::Problem problem;
                ceres::Solver::Summary summary;
                ceres::Solver::Options options;
                options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
                options.num_threads                = 4;
                options.max_num_iterations         = 50;

                // 参数块
                // add parameter blocks
                for (size_t k = 0; k <= preintegrationlist.size(); k++) {
                    // 位姿
                    ceres::LocalParameterization *parameterization = new (PoseParameterization);
                    problem.AddParameterBlock(statedatalist[k].pose, Preintegration::numPoseParameter(),
                                              parameterization);

                    problem.AddParameterBlock(statedatalist[k].mix,
                                              Preintegration::numMixParameter(preintegration_options));
                }

                // vlp残差
                // vlp factors
                // Add outlier culling as you need
                int index = 0;
                for (auto &data : vlplist) {
                    if (scheme == 1){
                        auto factor = new VLPFactor(data, antlever);
                        for (size_t i = index; i <= preintegrationlist.size(); ++i) {
                            if (fabs(data.time - timelist[i]) < MINIMUM_INTERVAL) {
                                problem.AddResidualBlock(factor, nullptr, statedatalist[i].pose);

                                index++;
                                break;
                            }
                        }
                    } else {
                        auto factor = new VLPFactor2(data, antlever, Nled, vlp_power, M, LED);
                        for (size_t i = index; i <= preintegrationlist.size(); ++i) {
                            if (fabs(data.time - timelist[i]) < MINIMUM_INTERVAL) {
                                problem.AddResidualBlock(factor, nullptr, statedatalist[i].pose);

                                index++;
                                break;
                            }
                        }
                    }
                }

                // 预积分残差
                // preintegration factors
                for (size_t k = 0; k < preintegrationlist.size(); k++) {
                    auto factor = new PreintegrationFactor(preintegrationlist[k]);
                    problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix,
                                             statedatalist[k + 1].pose, statedatalist[k + 1].mix);
                }

                // NHC factors
                for (size_t k = 0; k <= preintegrationlist.size(); k++) {
                    auto factor = new NHCFactor(antlever);
                    problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix);
                }

                {
                    // IMU误差控制
                    // add IMU bias-constraint factors
                    auto factor = new ImuErrorFactor(*preintegrationlist.rbegin());
                    problem.AddResidualBlock(factor, nullptr, statedatalist[preintegrationlist.size()].mix);
                }

                // 边缘化残差
                // prior factor
/*                 if (last_marginalization_info && last_marginalization_info->isValid()) {
                    auto factor = new MarginalizationFactor(last_marginalization_info);
                    problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
                } */

                // 求解最小二乘
                // solve the Least-Squares problem
                solver.Solve(options, &problem, &summary);
                //                std::cout << sow - 1 << ": " << summary.BriefReport() << std::endl;

                // 输出进度
                // output the percentage
                int percent            = ((int) sow - starttime) * 100 / (endtime - starttime);
                static int lastpercent = 0;
                if (abs(percent - lastpercent) >= 1) {
                    lastpercent = percent;
                    std::cout << "Percentage: " << std::setw(3) << percent << "%\r";
                    flush(std::cout);
                }
            }

            if (preintegrationlist.size() == static_cast<size_t>(windows)) {
/*                 {
                    // 边缘化
                    // marginalization
                    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();
                    if (last_marginalization_info && last_marginalization_info->isValid()) {

                        std::vector<int> marginilized_index;
                        for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++) {
                            if (last_marginalization_parameter_blocks[k] == statedatalist[0].pose ||
                                last_marginalization_parameter_blocks[k] == statedatalist[0].mix) {
                                marginilized_index.push_back(static_cast<int>(k));
                            }
                        }

                        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr, last_marginalization_parameter_blocks, marginilized_index);
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // IMU残差
                    // preintegration factors
                    {
                        auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist[0]);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr,
                            std::vector<double *>{statedatalist[0].pose, statedatalist[0].mix, statedatalist[1].pose,
                                                  statedatalist[1].mix},
                            std::vector<int>{0, 1});
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // vlp残差
                    // vlp factors
                    {
                        if (fabs(timelist[0] - vlplist[0].time) < MINIMUM_INTERVAL) {
                            if(scheme == 1) {
                                auto factor = std::make_shared<VLPFactor>(vlplist[0], antlever);
                                auto residual = std::make_shared<ResidualBlockInfo>(
                                    factor, nullptr, std::vector<double *>{statedatalist[0].pose}, std::vector<int>{});
                                marginalization_info->addResidualBlockInfo(residual);
                            } else {
                                auto factor = std::make_shared<VLPFactor2>(vlplist[0], antlever, vlp_power, M);
                                auto residual = std::make_shared<ResidualBlockInfo>(
                                    factor, nullptr, std::vector<double *>{statedatalist[0].pose}, std::vector<int>{});
                                marginalization_info->addResidualBlockInfo(residual);
                            }
                        }
                    }

                    // 边缘化处理
                    // do marginalization
                    marginalization_info->marginalization();

                    // 数据指针调整
                    // get new pointers
                    std::unordered_map<long, double *> address;
                    for (size_t k = 1; k <= preintegrationlist.size(); k++) {
                        address[reinterpret_cast<long>(statedatalist[k].pose)] = statedatalist[k - 1].pose;
                        address[reinterpret_cast<long>(statedatalist[k].mix)]  = statedatalist[k - 1].mix;
                    }
                    last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
                    last_marginalization_info             = std::move(marginalization_info);
                } */

                // 滑窗处理
                // sliding window
                {
                    if (lround(timelist[0]) == lround(vlplist[0].time)) {
                        vlplist.erase(vlplist.begin());
                    }
                    timelist.erase(timelist.begin());
                    preintegrationlist.erase(preintegrationlist.begin());

                    for (int k = 0; k < windows; k++) {
                        statedatalist[k] = statedatalist[k + 1];
                        statelist[k]     = Preintegration::stateFromData(statedatalist[k], preintegration_options);
                    }
                    statelist[windows] = Preintegration::stateFromData(statedatalist[windows], preintegration_options);
                    state_curr         = statelist[windows];
                }
            } else {
                state_curr =
                    Preintegration::stateFromData(statedatalist[preintegrationlist.size()], preintegration_options);
            }

            // write result
            writeNavResult(*timelist.rbegin()-starttime, state_curr, navfile, errfile);

            // 新建立新的预积分
            // build a new preintegration object
            preintegrationlist.emplace_back(
                Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));
        } else {
            auto integration = *preintegrationlist.rbegin();
            writeNavResult(integration->endTime()-starttime, integration->currentState(), navfile, errfile);
        }
    }

    navfile.close();
    errfile.close();
    imufile.close();
    vlpfile.close();

    auto te = absl::Now();
    std::cout << "\r\nCost " << absl::ToDoubleSeconds(te - ts) << " s in total\n";

    return 0;
}

void writeNavResult(double time, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile) {
    static int counts = 0;
    if ((counts++ % 10) != 0) {
        return;
    }

    vector<double> result;

    Vector3d pos = state.p;
    Vector3d att = Rotation::quaternion2euler(state.q) * R2D;
    Vector3d vel = state.v;
    Vector3d bg  = state.bg * R2D * 3600;
    Vector3d ba  = state.ba * 1e5;

    {
        result.clear();

        result.push_back(time);
        result.push_back(pos[0]);
        result.push_back(pos[1]);
        result.push_back(pos[2]);
        result.push_back(vel[0]);
        result.push_back(vel[1]);
        result.push_back(vel[2]);
        result.push_back(att[0]);
        result.push_back(att[1]);
        result.push_back(att[2]);
        navfile.dump(result);
    }

    {
        result.clear();

        result.push_back(time);
        result.push_back(bg[0]);
        result.push_back(bg[1]);
        result.push_back(bg[2]);
        result.push_back(ba[0]);
        result.push_back(ba[1]);
        result.push_back(ba[2]);
        result.push_back(state.sodo);
        errfile.dump(result);
    }
}

void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid) {
    double time = mid;

    double scale = (imu01.time - time) / imu01.dt;
    IMU buff     = imu01;

    imu00.time   = time;
    imu00.dt     = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel   = buff.dvel * (1 - scale);
    imu00.odovel = buff.odovel * (1 - scale);

    imu11.time   = buff.time;
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
}

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    double time = mid;

    if (imu0.time < time && imu1.time > time) {
        double dt = time - imu0.time;

        // 前一个历元接近
        // close to the first epoch
        if (dt < 0.0001) {
            return -1;
        }

        // 后一个历元接近
        // close to the second epoch
        dt = imu1.time - time;
        if (dt < 0.0001) {
            return 1;
        }

        // 需内插
        // need interpolation
        return 2;
    }

    return 0;
}
