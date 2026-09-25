#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef struct VLP {
    double time;

    Vector3d xyz;
    Vector3d std;
    std::vector<double> RSS;
    std::vector<double> RSS_std;
} VLP;

typedef struct IMU {
    double time;
    double dt;

    Vector3d dtheta;
    Vector3d dvel;

    double odovel;
} IMU;

typedef struct Pose {
    Matrix3d R;
    Vector3d t;
} Pose;

#endif // TYPES_H
