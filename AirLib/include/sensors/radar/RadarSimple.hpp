#pragma once

#include <random>
#include <vector>
#include <cmath>
#include "common/Common.hpp"
#include "sensors/radar/RadarBase.hpp"
#include "sensors/radar/RadarSimpleParams.hpp"
#include "common/ClockBase.hpp"
#include "common/RandomGenerator.hpp"
#include "common/AirSimSettings.hpp"  // ✅ RadarSetting 구조체 포함
#include "physics/PhysicsBody.hpp"

namespace msr { namespace airlib {

class RadarSimple : public RadarBase {
public:
    RadarSimple(const std::string& sensor_name, const RadarSimpleParams& params)
        : RadarBase(sensor_name), params_(params)
    {
        last_update_time_ = clock()->nowNanos();
    }

    virtual void resetImplementation() override
    {
        detections_.clear();
        last_update_time_ = clock()->nowNanos();
    }

    virtual void initializeSensor(const SensorSetting* sensor_setting) override
    {
        RadarBase::initializeSensor(sensor_setting);

        auto radar_setting = static_cast<const AirSimSettings::RadarSetting*>(sensor_setting);
        params_.initializeFromSettings(*radar_setting);
        last_update_time_ = clock()->nowNanos();
    }

    virtual void update() override
    {
        updateDetections();
    }

    virtual std::vector<RadarDetection> getDetections() const override
    {
        return detections_;
    }

private:
    RadarSimpleParams params_;
    std::vector<RadarDetection> detections_;
    TTimePoint last_update_time_;
    RandomGenerator rand_gen_;

    void updateDetections()
    {
        detections_.clear();
        
        // 레이 캐스팅을 통한 물체 감지 구현
        // AirSim의 물리 엔진을 사용하여 레이 캐스트 수행
        // 속도 계산 및 레이더 감지 정보 생성
    }

    void generateRadarData()
    {
        detections_.clear();

        for (int i = 0; i < static_cast<int>(params_.points_per_scan); ++i) {
            float azimuth = rand_gen_.uniformReal(-params_.azimuth_fov / 2.0f, params_.azimuth_fov / 2.0f) * M_PIf / 180.0f;
            float elevation = rand_gen_.uniformReal(-params_.altitude_fov / 2.0f, params_.altitude_fov / 2.0f) * M_PIf / 180.0f;

            RadarDetection detection;
            detection.depth = params_.range_max * rand_gen_.uniformReal(0.0f, 1.0f);
            detection.azimuth = azimuth;
            detection.altitude = elevation;
            detection.velocity = rand_gen_.uniformReal(-5.0f, 5.0f);

            detections_.push_back(detection);
        }
    }

    void addNoise(RadarDetection& detection)
    {
        if (params_.noise_std_dev > 0.0f) {
            detection.depth += rand_gen_.gaussianFloat(0, params_.noise_std_dev);
            detection.velocity += rand_gen_.gaussianFloat(0, params_.noise_std_dev);
        }
    }
};

}} // namespace msr::airlib
