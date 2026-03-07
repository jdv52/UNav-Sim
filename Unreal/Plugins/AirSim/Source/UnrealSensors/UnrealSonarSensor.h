#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/sonar/SonarSimple.hpp"
#include "NedTransform.h"

// UnrealLidarSensor implementation that uses Ray Tracing in Unreal.
// The implementation uses a model similar to CARLA Lidar implementation.
// Thanks to CARLA folks for this.
class UnrealSonarSensor : public msr::airlib::SonarSimple
{
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealSonarSensor(const AirSimSettings::SonarSetting& setting,
                      AActor* actor, const NedTransform* ned_transform);

protected:
    virtual void getPointCloud(const msr::airlib::Pose& sonar_pose, const msr::airlib::Pose& vehicle_pose,
                               msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud) override;

private:
    using Vector3r = msr::airlib::Vector3r;
    using VectorMath = msr::airlib::VectorMath;

    void createLasers();
    bool shootLaser(const msr::airlib::Pose& sonar_pose, const msr::airlib::Pose& vehicle_pose,
                    const float horizontal_angle, const float vertical_angle,
                    const msr::airlib::SonarSimpleParams& params, Vector3r& point, int& segmentationID);

private:
    AActor* actor_;
    const NedTransform* ned_transform_;

    msr::airlib::vector<msr::airlib::real_T> laser_angles_;
    float current_horizontal_angle_ = 0.0f;
};