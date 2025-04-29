#ifndef msr_airlib_SensorFactoryBase_hpp
#define msr_airlib_SensorFactoryBase_hpp

#include "SensorBase.hpp"
#include "SensorCollection.hpp"
#include <memory>

// sensors
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/magnetometer/MagnetometerSimple.hpp"
#include "sensors/gps/GpsSimple.hpp"
#include "sensors/barometer/BarometerSimple.hpp"
#include "sensors/radar/RadarSimple.hpp"   // ✅ RadarSimple 추가

namespace msr
{
namespace airlib
{

    class SensorFactory
    {
    public:
        // creates one sensor from settings
        virtual std::shared_ptr<SensorBase> createSensorFromSettings(
            const SensorBase::SensorSetting* sensor_setting) const
        {
            if (!sensor_setting)
                return nullptr;

            switch (sensor_setting->sensor_type) {
            case SensorBase::SensorType::Imu: {
                auto imu_setting = static_cast<const AirSimSettings::ImuSetting*>(sensor_setting);
                return std::make_shared<ImuSimple>(*imu_setting);
            }
            case SensorBase::SensorType::Magnetometer: {
                auto mag_setting = static_cast<const AirSimSettings::MagnetometerSetting*>(sensor_setting);
                return std::make_shared<MagnetometerSimple>(*mag_setting);
            }
            case SensorBase::SensorType::Gps: {
                auto gps_setting = static_cast<const AirSimSettings::GpsSetting*>(sensor_setting);
                return std::make_shared<GpsSimple>(*gps_setting);
            }
            case SensorBase::SensorType::Barometer: {
                auto baro_setting = static_cast<const AirSimSettings::BarometerSetting*>(sensor_setting);
                return std::make_shared<BarometerSimple>(*baro_setting);
            }
            case SensorBase::SensorType::Radar: {
                auto radar_setting = static_cast<const AirSimSettings::RadarSetting*>(sensor_setting);
                RadarSimpleParams params;
                params.initializeFromSettings(*radar_setting);
                return std::make_shared<RadarSimple>(radar_setting->sensor_name, params);
            }
            default:
                throw std::invalid_argument("Unknown sensor type");
            }
        }

        // creates sensor-collection
        virtual void createSensorsFromSettings(
            const std::map<std::string, std::shared_ptr<SensorBase::SensorSetting>>& sensors_settings,
            SensorCollection& sensors,
            vector<shared_ptr<SensorBase>>& sensor_storage) const
        {
            for (const auto& sensor_setting_pair : sensors_settings) {
                const SensorBase::SensorSetting* sensor_setting = sensor_setting_pair.second.get();

                if (sensor_setting == nullptr || !sensor_setting->enabled)
                    continue;

                std::shared_ptr<SensorBase> sensor = createSensorFromSettings(sensor_setting);
                if (sensor) {
                    sensor_storage.push_back(sensor);
                    sensors.insert(sensor.get(), sensor_setting->sensor_type);
                }
            }
        }

        virtual ~SensorFactory() = default;
    };

}
} //namespace
#endif
