#ifndef PREINTEGRATION_H
#define PREINTEGRATION_H

#include "preintegration_base.h"
#include "preintegration_earth.h"
#include "preintegration_earth_odo.h"
#include "preintegration_normal.h"
#include "preintegration_odo.h"

class Preintegration {

public:
    Preintegration() = default;

    enum PreintegrationOptions {
        PREINTEGRATION_NORMAL    = 0,
        PREINTEGRATION_ODO       = 1,
        PREINTEGRATION_EARTH     = 2,
        PREINTEGRATION_EARTH_ODO = 3,
    };

    static PreintegrationOptions getOptions(bool isuseodo) {
        int options = PREINTEGRATION_NORMAL;

        if (isuseodo) {
            options += PREINTEGRATION_ODO;
        }

        return static_cast<PreintegrationOptions>(options);
    }

    static std::shared_ptr<PreintegrationBase>
        createPreintegration(const std::shared_ptr<IntegrationParameters> &parameters, const IMU &imu0,
                             const IntegrationState &state, PreintegrationOptions options) {
        std::shared_ptr<PreintegrationBase> preintegration;

        if (options == PREINTEGRATION_NORMAL) {
            preintegration = std::make_shared<PreintegrationNormal>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_ODO) {
            preintegration = std::make_shared<PreintegrationOdo>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_EARTH) {
            preintegration = std::make_shared<PreintegrationEarth>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            preintegration = std::make_shared<PreintegrationEarthOdo>(parameters, imu0, state);
        }

        return preintegration;
    }

    static int numPoseParameter() {
        return PreintegrationBase::NUM_POSE;
    }

    static IntegrationStateData stateToData(const IntegrationState &state, PreintegrationOptions options) {
        if (options == PREINTEGRATION_NORMAL) {
            return PreintegrationNormal::stateToData(state);
        } else if (options == PREINTEGRATION_ODO) {
            return PreintegrationOdo::stateToData(state);
        } else if (options == PREINTEGRATION_EARTH) {
            return PreintegrationEarth::stateToData(state);
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            return PreintegrationEarthOdo::stateToData(state);
        }
        return {};
    }

    static IntegrationState stateFromData(const IntegrationStateData &data, PreintegrationOptions options) {
        if (options == PREINTEGRATION_NORMAL) {
            return PreintegrationNormal::stateFromData(data);
        } else if (options == PREINTEGRATION_ODO) {
            return PreintegrationOdo::stateFromData(data);
        } else if (options == PREINTEGRATION_EARTH) {
            return PreintegrationEarth::stateFromData(data);
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            return PreintegrationEarthOdo::stateFromData(data);
        }

        return {};
    }

    static int numMixParameter(PreintegrationOptions options) {
        int num = 0;
        if (options == PREINTEGRATION_NORMAL) {
            num = PreintegrationNormal::NUM_MIX;
        } else if (options == PREINTEGRATION_ODO) {
            num = PreintegrationOdo::NUM_MIX;
        } else if (options == PREINTEGRATION_EARTH) {
            num = PreintegrationEarth::NUM_MIX;
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            num = PreintegrationEarthOdo::NUM_MIX;
        }
        return num;
    }
};

#endif // PREINTEGRATION_H
