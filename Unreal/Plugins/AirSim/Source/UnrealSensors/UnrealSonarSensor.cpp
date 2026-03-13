// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealSonarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

// ctor
UnrealSonarSensor::UnrealSonarSensor(const AirSimSettings::SonarSetting& setting,
                                     AActor* actor, const NedTransform* ned_transform)
    : SonarSimple(setting), actor_(actor), ned_transform_(ned_transform)
{
    
}

// returns a point-cloud for the tick
void UnrealSonarSensor::getPointCloud(const msr::airlib::Pose& sonar_pose, const msr::airlib::Pose& vehicle_pose,
                                      const msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& image,
                                      msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{
    image.clear();

    const msr::airlib::SonarSimpleParams params = getParams();

    // get sonar pose and orientation
    Vector3r dummy;

    /*
    image.assign(params.num_azimuth_bins * params.num_range_bins, FLT_MAX);

    ParallelFor(
        azimuth_angles.size(), [&](int32 i) {
            for (auto v : elevation_angles) {
                // detection info
                if (shootBeam(
                        sonar_pose, vehicle_pose, azimuth_angles[i], v, params, dummy)) {
                    // get intensity and populate histogram
                }
            }
        },
        EParallelForFlags::Unbalanced);
    */
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealSonarSensor::shootBeam(const msr::airlib::Pose& sonar_pose, const msr::airlib::Pose& vehicle_pose,
                                  const float azimuth_angle, const float elevation_angle,
                                  const msr::airlib::SonarSimpleParams& params, Vector3r& point)
{
    /*
    Vector3r start = VectorMath::add(sonar_pose, vehicle_pose).position;

    Quaternionr ray_q_l = msr::airlib::VectorMath::toQuaternion(
        msr::airlib::Utils::degreesToRadians(elevation_angle),
        0,
        msr::airlib::Utils::degreesToRadians(azimuth_angle));

    // get ray quaternion in body frame
    msr::airlib::Quaternionr ray_q_b = VectorMath::coordOrientationAdd(ray_q_l, sonar_pose.orientation);

    // get ray quaternion in world frame
    msr::airlib::Quaternionr ray_q_w = VectorMath::coordOrientationAdd(ray_q_b, vehicle_pose.orientation);

    // get ray vector (end position)
    Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range_max s+ start;

    FHitResult hit_result = FHitResult(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result, actor_, ECC_Visibility);

    if (is_hit) {
        // decide the frame for the point-cloud
        switch (params.data_frame) {
        case AirSimSettings::SonarSetting::DataFrame::VehicleInertialFrame:
            // current detault behavior; though it is probably not very useful.
            // not changing the default for now to maintain backwards-compat.
            point = ned_transform_->toLocalNed(hit_result.ImpactPoint);
            break;
        case AirSimSettings::SonarSetting::DataFrame::SensorLocalFrame:
            // point in vehicle intertial frame
            Vector3r point_v_i = ned_transform_->toLocalNed(hit_result.ImpactPoint);

            // tranform to lidar frame
            point = VectorMath::transformToBodyFrame(point_v_i, sonar_pose + vehicle_pose, true);

            break;
        }

        return true;
    }
    else {
        return false;
    }
    */
    return false;
}
