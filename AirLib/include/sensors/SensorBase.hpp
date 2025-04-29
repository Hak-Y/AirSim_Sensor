// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "common/UpdatableObject.hpp"
#include "common/CommonStructs.hpp"
#include "common/Settings.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include <string>

namespace msr
{
namespace airlib
{

// Forward declarations
struct AirSimSettings;
struct SensorSetting;

    /*
    Derived classes should not do any work in constructor which requires ground truth.
    After construction of the derived class an initialize(...) must be made which would
    set the sensor in good-to-use state by call to reset.
*/
    class SensorBase : public UpdatableObject
    {
    public:
        enum class SensorType {
            Imu,
            Magnetometer,
            Gps,
            Barometer,
            Distance,
            Lidar,
            Radar
        };

        struct SensorSetting {
            SensorType sensor_type;
            std::string sensor_name;
            bool enabled = true;
            Settings settings;
            
            virtual ~SensorSetting() = default;
        };

        SensorBase(const std::string& sensor_name = "")
            : name_(sensor_name)
        {
        }

    protected:
        struct GroundTruth
        {
            const Kinematics::State* kinematics;
            const Environment* environment;
        };

    public:
        virtual void initialize(const Kinematics::State* kinematics, const Environment* environment)
        {
            ground_truth_.kinematics = kinematics;
            ground_truth_.environment = environment;
        }
    
        // ✅ 여기에 추가
        virtual void initializeSensor(const SensorSetting* sensor_setting)
        {
            (void)sensor_setting;
        }

        const GroundTruth& getGroundTruth() const
        {
            return ground_truth_;
        }

        const std::string& getName() const
        {
            return name_;
        }

        virtual ~SensorBase() = default;

    private:
        //ground truth can be shared between many sensors
        GroundTruth ground_truth_ = { nullptr, nullptr };
        std::string name_ = "";
    };
}
} //namespace
