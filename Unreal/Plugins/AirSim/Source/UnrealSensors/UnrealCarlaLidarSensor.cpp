// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealCarlaLidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "common/VectorMath.hpp"
#include "common/common_utils/Utils.hpp"
#include "sensors/Carla_lidar/CarlaLidarSimpleParams.hpp"
// #include "sensors/Carla_lidar/CarlaLidarSimple.hpp"

using namespace msr::airlib;

UnrealCarlaLidarSensor::UnrealCarlaLidarSensor(const AirSimSettings::CarlaLidarSetting& setting,
                                               AActor* actor, const NedTransform* ned_transform)
    : CarlaLidar(setting), actor_(actor), ned_transform_(ned_transform)
{
    createLasers();
}

void UnrealCarlaLidarSensor::createLasers()
{
    CarlaLidarParams params = getParams();
    const auto number_of_lasers = params.channels;
    if (number_of_lasers <= 0) return;

    float delta_angle = 0;
    if (number_of_lasers > 1)
        delta_angle = (params.upper_fov - params.lower_fov) / static_cast<float>(number_of_lasers - 1);

    laser_angles_.clear();
    for (uint32_t i = 0; i < number_of_lasers; ++i) {
        float vertical_angle = params.upper_fov - static_cast<float>(i) * delta_angle;
        laser_angles_.emplace_back(vertical_angle);
    }
}

void UnrealCarlaLidarSensor::getPointCloud(const Pose& lidar_pose, const Pose& vehicle_pose,
                                           TTimeDelta delta_time, vector<real_T>& point_cloud, vector<int>& segmentation_cloud)
{
    point_cloud.clear();
    segmentation_cloud.clear();

    CarlaLidarParams params = getParams();
    const auto number_of_lasers = params.channels;

    UAirBlueprintLib::LogMessageString("CarlaLidar", "Starting point cloud generation", LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("CarlaLidar", std::string(TCHAR_TO_UTF8(*FString::Printf(TEXT("Number of lasers: %d"), number_of_lasers))), LogDebugLevel::Informational);

    constexpr float MAX_POINTS_IN_SCAN = 1e5f;
    uint32_t total_points_to_scan = FMath::RoundHalfFromZero(params.points_per_second * delta_time);
    if (total_points_to_scan > MAX_POINTS_IN_SCAN) {
        total_points_to_scan = MAX_POINTS_IN_SCAN;
        UAirBlueprintLib::LogMessageString("Lidar", "Capping number of points", LogDebugLevel::Failure);
    }

    uint32_t points_per_laser = FMath::RoundHalfFromZero(total_points_to_scan / float(number_of_lasers));
    if (points_per_laser <= 0) {
        UAirBlueprintLib::LogMessageString("CarlaLidar", "No points to scan - points_per_laser is 0", LogDebugLevel::Failure);
        return;
    }

    float angle_distance = params.rotation_frequency * 360.0f * delta_time;
    float angle_per_point = angle_distance / points_per_laser;

    float laser_start = std::fmod(360.0f + params.upper_fov, 360.0f);
    float laser_end   = std::fmod(360.0f + params.lower_fov, 360.0f);

    UAirBlueprintLib::LogMessageString("CarlaLidar", std::string(TCHAR_TO_UTF8(*FString::Printf(TEXT("Laser angles: start=%.2f, end=%.2f"), laser_start, laser_end))), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("CarlaLidar", std::string(TCHAR_TO_UTF8(*FString::Printf(TEXT("Points per laser: %d"), points_per_laser))), LogDebugLevel::Informational);

    uint32_t total_points_generated = 0;
    uint32_t total_points_skipped = 0;

    for (uint32_t laser = 0; laser < number_of_lasers; ++laser) {
        float vertical_angle = laser_angles_[laser];
        uint32_t points_generated_for_laser = 0;

        for (uint32_t i = 0; i < points_per_laser; ++i) {
            float horizontal_angle = std::fmod(current_horizontal_angle_ + angle_per_point * i, 360.0f);

            if (!VectorMath::isAngleBetweenAngles(horizontal_angle, laser_start, laser_end)) {
                total_points_skipped++;
                continue;
            }

            Vector3r point;
            int segmentationID = -1;
            if (shootLaser(lidar_pose, vehicle_pose, laser, horizontal_angle, vertical_angle, params, point, segmentationID)) {
                point_cloud.push_back(point.x());
                point_cloud.push_back(point.y());
                point_cloud.push_back(point.z());
                segmentation_cloud.push_back(segmentationID);
                points_generated_for_laser++;
                total_points_generated++;
            }
        }

        if (points_generated_for_laser == 0) {
            UAirBlueprintLib::LogMessageString("CarlaLidar", std::string(TCHAR_TO_UTF8(*FString::Printf(TEXT("Laser %d generated no points"), laser))), LogDebugLevel::Failure);
        }
    }

    if (total_points_generated == 0) {
        UAirBlueprintLib::LogMessageString("CarlaLidar", "No points were generated in this scan", LogDebugLevel::Failure);
        UAirBlueprintLib::LogMessageString("CarlaLidar", std::string(TCHAR_TO_UTF8(*FString::Printf(TEXT("Total points skipped: %d"), total_points_skipped))), LogDebugLevel::Failure);
    } else {
        UAirBlueprintLib::LogMessageString("CarlaLidar", std::string(TCHAR_TO_UTF8(*FString::Printf(TEXT("Generated %d points"), total_points_generated))), LogDebugLevel::Informational);
        UAirBlueprintLib::LogMessageString("CarlaLidar", std::string(TCHAR_TO_UTF8(*FString::Printf(TEXT("First point: (%.2f, %.2f, %.2f)"), 
            point_cloud[0], point_cloud[1], point_cloud[2]))), LogDebugLevel::Informational);
    }

    current_horizontal_angle_ = std::fmod(current_horizontal_angle_ + angle_distance, 360.0f);
}

bool UnrealCarlaLidarSensor::shootLaser(const Pose& lidar_pose, const Pose& vehicle_pose,
                                        uint32 laser, float horizontal_angle, float vertical_angle,
                                        const CarlaLidarParams params, Vector3r& point, int& segmentationID)
{
    Vector3r start = VectorMath::add(lidar_pose, vehicle_pose).position;

    Quaternionr ray_q_l = VectorMath::toQuaternion(
        Utils::degreesToRadians(vertical_angle), 0.0f, Utils::degreesToRadians(horizontal_angle));
    Quaternionr ray_q_b = VectorMath::coordOrientationAdd(ray_q_l, lidar_pose.orientation);
    Quaternionr ray_q_w = VectorMath::coordOrientationAdd(ray_q_b, vehicle_pose.orientation);

    Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range + start;

    FHitResult hit_result(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start),
                                                ned_transform_->fromLocalNed(end), hit_result, actor_, ECC_Visibility);

    if (is_hit) {
        AActor* hit_actor = hit_result.GetActor();
        if (hit_actor != nullptr) {
            TArray<UMeshComponent*> components;
            hit_actor->GetComponents<UMeshComponent>(components);
            for (auto* comp : components) {
                segmentationID = (segmentationID == -1) ? comp->CustomDepthStencilValue : segmentationID;
            }
        }

        if (params.data_frame == AirSimSettings::kVehicleInertialFrame) {
            point = ned_transform_->toLocalNed(hit_result.ImpactPoint);
        }
        else if (params.data_frame == AirSimSettings::kSensorLocalFrame) {
            Vector3r point_v_i = ned_transform_->toLocalNed(hit_result.ImpactPoint);
            point = VectorMath::transformToBodyFrame(point_v_i, lidar_pose + vehicle_pose, true);
        }
        else {
            throw std::runtime_error("Unknown data_frame specified in Lidar settings.");
        }

        return true;
    }

    return false;
}
