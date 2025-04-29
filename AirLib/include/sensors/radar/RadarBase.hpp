#pragma once

#include <vector>
#include "common/Common.hpp"
#include "sensors/SensorBase.hpp"

namespace msr { namespace airlib {

struct RadarDetection {
    real_T depth;        // 물체까지의 거리 (미터)
    real_T azimuth;      // 수평각 (라디안)
    real_T altitude;     // 고도각 (라디안)
    real_T velocity;     // 상대 속도 (m/s)

    RadarDetection() : depth(0), azimuth(0), altitude(0), velocity(0) {}
    RadarDetection(real_T d, real_T az, real_T alt, real_T vel)
        : depth(d), azimuth(az), altitude(alt), velocity(vel) {}
};

class RadarBase : public SensorBase {
public:
    RadarBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name) {}

    virtual ~RadarBase() = default;

    // 레이더 데이터를 가져오는 가상 함수
    virtual std::vector<RadarDetection> getDetections() const = 0;

    virtual void update() = 0;
};

}} // namespace msr::airlib
