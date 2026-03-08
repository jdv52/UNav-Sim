#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/dvl/DvlSimple.hpp"
#include "NedTransform.h"

class UnrealDvlSensor : public msr::airlib::DvlSimple
{
    using Vector3r = msr::airlib::Vector3r;
    using Pose = msr::airlib::Pose;

public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealDvlSensor(const AirSimSettings::DvlSetting& setting,
                    AActor* actor, const NedTransform* ned_transform);

protected:
    virtual void getBeamScans(const Pose& dvl_pose, const Pose& vehicle_pose,
                              msr::airlib::TTimeDelta delta_time, std::array<Vector3r, 4>& beam_unit_vecs, std::array<float, 4>& beam_ranges) override;

    bool shootBeam(const Pose& dvl_pose, const Pose& vehicle_pose,
                   const Vector3r& beam_vector, float& range, const msr::airlib::DvlSimpleParams& params);

private:
    void createBeams();

    AActor* actor_;
    const NedTransform* ned_transform_;
    std::array<Vector3r, 4> beam_unit_vectors_;
};