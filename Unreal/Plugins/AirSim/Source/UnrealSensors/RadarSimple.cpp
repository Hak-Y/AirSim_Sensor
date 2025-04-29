#include "RadarSimple.hpp"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetMathLibrary.h"

RadarSimple::RadarSimple()
    : sensor_component_(nullptr),
      last_update_time_(0.0f),
      update_interval_(0.05f), // 20Hz
      points_per_second_(1500),
      max_distance_(1000.0f),
      horizontal_fov_(30.0f),
      vertical_fov_(10.0f),
      noise_std_dev_(0.1f)
{
}

void RadarSimple::initialize(USceneComponent* sensor_component)
{
    sensor_component_ = sensor_component;
    last_update_time_ = 0.0f;
}

void RadarSimple::update(float delta_time)
{
    last_update_time_ += delta_time;
    if (last_update_time_ >= update_interval_) {
        generateRadarData();
        last_update_time_ = 0.0f;
    }
}

void RadarSimple::generateRadarData()
{
    detections_.Empty();

    if (!sensor_component_)
        return;

    const FVector start_location = sensor_component_->GetComponentLocation();
    const FQuat orientation = sensor_component_->GetComponentQuat();
    UWorld* world = sensor_component_->GetWorld();
    if (!world)
        return;

    int total_rays = static_cast<int>(points_per_second_ * update_interval_);

    for (int i = 0; i < total_rays; ++i) {
        float azimuth = FMath::RandRange(-horizontal_fov_/2.0f, horizontal_fov_/2.0f) * PI / 180.0f;
        float elevation = FMath::RandRange(-vertical_fov_/2.0f, vertical_fov_/2.0f) * PI / 180.0f;

        FVector direction = orientation.RotateVector(FVector(
            FMath::Cos(elevation) * FMath::Cos(azimuth),
            FMath::Cos(elevation) * FMath::Sin(azimuth),
            FMath::Sin(elevation)
        ));

        FVector end_location = start_location + (direction * max_distance_);

        FHitResult hit_result;
        FCollisionQueryParams query_params(SCENE_QUERY_STAT(RadarTrace), true);
        bool hit = world->LineTraceSingleByChannel(
            hit_result,
            start_location,
            end_location,
            ECC_Visibility,
            query_params
        );

        if (hit) {
            RadarDetection detection;

            FVector hit_vector = hit_result.ImpactPoint - start_location;
            detection.depth = hit_vector.Size();
            detection.azimuth = FMath::Atan2(hit_vector.Y, hit_vector.X);
            detection.altitude = FMath::Atan2(hit_vector.Z, FVector(hit_vector.X, hit_vector.Y, 0).Size());

            FVector target_velocity = hit_result.GetActor() ? hit_result.GetActor()->GetVelocity() : FVector::ZeroVector;
            FVector relative_velocity = target_velocity;
            detection.velocity = FVector::DotProduct(relative_velocity, direction);

            addNoise(detection);
            detections_.Add(detection);
        }
    }
}

void RadarSimple::addNoise(RadarDetection& detection)
{
    if (noise_std_dev_ > 0.0f) {
        detection.depth += FMath::FRandRange(-noise_std_dev_, noise_std_dev_);
        detection.velocity += FMath::FRandRange(-noise_std_dev_, noise_std_dev_);
    }
}

const TArray<RadarDetection>& RadarSimple::getDetections() const
{
    return detections_;
}
