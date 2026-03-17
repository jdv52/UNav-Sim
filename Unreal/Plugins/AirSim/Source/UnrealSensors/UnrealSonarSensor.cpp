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
    point_cloud.clear();

    const msr::airlib::SonarSimpleParams params = getParams();

    // get sonar pose and orientation
    Vector3r dummy;

    if (params.return_image) {
        image.assign(azimuth_beam_count * range_bin_count, 0);
    }

    if (params.return_point_cloud) {
        point_cloud.assign(azimuth_beam_count * elevation_ray_count * 3, FLT_MAX);
    }

    ParallelFor(
        azimuth_beam_count, [&](int32 i) {
            for (int j = 0; j < elevation_ray_count; ++j) 
            {
                std::size_t base_index = (i * elevation_ray_count) + j;

                auto beam = getBeams()[base_index];

                FHitResult hit = FHitResult(ForceInit);

                if (shootBeam(
                        sonar_pose, vehicle_pose, beam, params, hit)) {
                    
                    if (params.return_image) {
                        FVector beam_direction = FVector(beam.x(), beam.y(), beam.z()).GetSafeNormal();
                        float z = 0.1f;
                        float r = (z - 0.1) / (z + 0.1);
                        float val = r * r * FVector::DotProduct(beam_direction, hit.ImpactNormal);
                        float dist_m = hit.Distance / 100.0f;

                        if (dist_m >= params.range_min && dist_m <= params.range_max) {
                            std::size_t range_bin = static_cast<std::size_t>(std::floor(((dist_m - params.range_min) / params.range_resolution)));
                            // image[i * range_bin_count + range_bin] += val;

                            // UAirBlueprintLib::LogMessageString("Range: " + std::to_string(dist_m), "", LogDebugLevel::Informational);
                            // UAirBlueprintLib::LogMessageString("Range_bin: " + std::to_string(range_bin), "", LogDebugLevel::Informational);
                        }
                    }

                    if (params.return_point_cloud) {
                        auto hit_location = ned_transform_->toLocalNed(hit.ImpactPoint);

                        auto point = VectorMath::transformToBodyFrame(hit_location, sonar_pose + vehicle_pose, true);

                        point_cloud[base_index * 3] = point.x();
                        point_cloud[base_index * 3 + 1] = point.y();
                        point_cloud[base_index * 3 + 2] = point.z();
                    }
                }
            }
        },
        EParallelForFlags::Unbalanced);

    // normalize histogram
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealSonarSensor::shootBeam(const msr::airlib::Pose& sonar_pose, const msr::airlib::Pose& vehicle_pose,
                                  const Vector3r& beam,
                                  const msr::airlib::SonarSimpleParams& params, FHitResult& hit)
{
    Vector3r start = VectorMath::add(sonar_pose, vehicle_pose).position;

    Vector3r end = start + VectorMath::rotateVector(beam, vehicle_pose.orientation, true);

    
    bool is_hit = UAirBlueprintLib::GetObstacle(
        actor_,
        ned_transform_->fromLocalNed(start),
        ned_transform_->fromLocalNed(end),
        hit,
        actor_,
        ECC_Visibility);

    return is_hit;
}
