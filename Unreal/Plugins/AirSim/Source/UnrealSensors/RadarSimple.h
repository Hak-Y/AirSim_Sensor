#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"

struct RadarDetection {
    float depth;
    float azimuth;
    float altitude;
    float velocity;
};

class RadarSimple
{
public:
    RadarSimple();

    void initialize(USceneComponent* sensor_component);
    void update(float delta_time);

    const TArray<RadarDetection>& getDetections() const;

private:
    void generateRadarData();
    void addNoise(RadarDetection& detection);

private:
    USceneComponent* sensor_component_;
    TArray<RadarDetection> detections_;
    float last_update_time_;
    float update_interval_;
    float points_per_second_;
    float max_distance_;
    float horizontal_fov_;
    float vertical_fov_;
    float noise_std_dev_;
};

