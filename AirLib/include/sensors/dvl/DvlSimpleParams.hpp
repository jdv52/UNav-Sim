#ifndef msr_airlib_DvlSimpleParams_hpp
#define msr_airlib_DvlSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr
{
namespace airlib
{

    struct DvlSimpleParams
    {
        real_T elevation_angle = 30.0;
        real_T range_max = 10.0;

        Pose relative_pose; // position and orientation

        bool return_ranges = true;
        bool draw_debug_lines = false;
        
        real_T update_frequency = 10; // Hz
        real_T startup_delay = 0; // sec

        real_T velocity_noise_stddev = 0.01f; // m/s
        real_T range_noise_stddev = 0.01f; // m

        void initializeFromSettings(const AirSimSettings::DvlSetting& settings)
        {
            std::string simmode_name = AirSimSettings::singleton().simmode_name;

            const auto& settings_json = settings.settings;
            elevation_angle = settings_json.getFloat("ElevationAngle", elevation_angle);
            range_max = settings_json.getFloat("RangeMax", range_max);
            draw_debug_lines = settings_json.getBool("DrawDebugLines", draw_debug_lines);
            return_ranges = settings_json.getBool("ReturnRanges", return_ranges);
            velocity_noise_stddev = settings_json.getFloat("SigmaVelocity", velocity_noise_stddev);
            range_noise_stddev = settings_json.getFloat("SigmaRange", range_noise_stddev);

            relative_pose.position = AirSimSettings::createVectorSetting(settings_json, VectorMath::nanVector());
            auto rotation = AirSimSettings::createRotationSetting(settings_json, AirSimSettings::Rotation::nanRotation());

            if (std::isnan(relative_pose.position.x()))
                relative_pose.position.x() = 0;
            if (std::isnan(relative_pose.position.y()))
                relative_pose.position.y() = 0;
            if (std::isnan(relative_pose.position.z())) {
                if (simmode_name == AirSimSettings::kSimModeTypeMultirotor || simmode_name == AirSimSettings::kSimModeTypeRov)
                    relative_pose.position.z() = 0;
                else
                    relative_pose.position.z() = -1; // a little bit above for cars
            }

            float pitch, roll, yaw;
            pitch = !std::isnan(rotation.pitch) ? rotation.pitch : 0;
            roll = !std::isnan(rotation.roll) ? rotation.roll : 0;
            yaw = !std::isnan(rotation.yaw) ? rotation.yaw : 0;
            relative_pose.orientation = VectorMath::toQuaternion(
                Utils::degreesToRadians(pitch), // pitch - rotation around Y axis
                Utils::degreesToRadians(roll), // roll  - rotation around X axis
                Utils::degreesToRadians(yaw)); // yaw   - rotation around Z axis


        }
    };
}
}

#endif