#pragma once

#include "common/AirSimSettings.hpp"  // RadarSetting 불러오기 위해 필요
#include "common/Common.hpp"

namespace msr { namespace airlib {

struct RadarSimpleParams {
    real_T range_max = 100.0f;          // 최대 감지 거리 (미터)
    real_T range_min = 0.5f;            // 최소 감지 거리 (미터)
    real_T azimuth_fov = 90.0f;         // 수평 시야각 (도)
    real_T altitude_fov = 20.0f;        // 수직 시야각 (도)
    
    uint32_t points_per_scan = 1000;    // 스캔당 포인트 수
    real_T update_frequency = 20.0f;    // 업데이트 주파수 (Hz)
    
    real_T noise_std_dev = 0.1f;        // 노이즈 표준편차
    bool draw_debug_points = false;      // 디버그 포인트 표시 여부

    void initializeFromSettings(const AirSimSettings::RadarSetting& setting)
    {
        range_max = setting.max_distance;
        azimuth_fov = setting.horizontal_fov;
        altitude_fov = setting.vertical_fov;
        update_frequency = 1.0f / setting.update_interval;
        points_per_scan = setting.points_per_second;
        // noise_std_dev는 기본값 사용
    }
};

}} // namespace msr::airlib
