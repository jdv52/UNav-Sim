// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealDvlSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

UnrealDvlSensor::UnrealDvlSensor(const AirSimSettings::DvlSetting& setting,
                                     AActor* actor, const NedTransform* ned_transform)
    : DvlSimple(setting), actor_(actor), ned_transform_(ned_transform)
{
    createBeams();
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealDvlSensor::shootBeam(const Pose& dvl_pose, const Pose& vehicle_pose,
                                   const Vector3r& beam_vector, float &range, const msr::airlib::DvlSimpleParams& params)
{
    Vector3r start = msr::airlib::VectorMath::add(dvl_pose, vehicle_pose).position;
    Vector3r end = start + beam_vector;

    FHitResult hit_result = FHitResult(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result, actor_, ECC_Visibility);

    if (is_hit) {
        Vector3r impact_vector = ned_transform_->toLocalNed(hit_result.ImpactPoint) - start;
        range = msr::airlib::VectorMath::magnitude(impact_vector);
    }
    
    return is_hit;
}

void UnrealDvlSensor::getBeamScans(const Pose& dvl_pose, const Pose& vehicle_pose,
    msr::airlib::TTimeDelta delta_time, std::array<Vector3r, 4>& beam_unit_vecs, std::array<float, 4>& beam_ranges)
{
    beam_unit_vecs = beam_unit_vectors_;

    const auto params = getParams();

    ParallelFor(4, [&](int32 idx) {
        const auto beam_vector = beam_unit_vectors_[idx] * params.range_max;
        if (shootBeam(dvl_pose, vehicle_pose, beam_vector, beam_ranges[idx], params)) {
            // can use result to mark bad reads
        }
    });

    return;
}

void UnrealDvlSensor::createBeams()
{   
    const auto params = getParams();

    float sin_elev_angle = sin(params.elevation_angle);
    float cos_elev_angle = cos(params.elevation_angle);

    beam_unit_vectors_[0] = { sin_elev_angle, 0, -cos_elev_angle };
    beam_unit_vectors_[1] = { 0, -sin_elev_angle, -cos_elev_angle };
    beam_unit_vectors_[2] = { -sin_elev_angle, 0, -cos_elev_angle };
    beam_unit_vectors_[3] = { 0, sin_elev_angle, -cos_elev_angle };
}