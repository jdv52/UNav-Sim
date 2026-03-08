#ifndef msr_airlib_Dvl_hpp
#define msr_airlib_Dvl_hpp

#include <random>
#include "common/Common.hpp"
#include "DvlSimpleParams.hpp"
#include "DvlBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr
{
namespace airlib
{

	class DvlSimple : public DvlBase
    {
    public:
        DvlSimple(const AirSimSettings::DvlSetting& setting = AirSimSettings::DvlSetting())
            : DvlBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);

            // additive noise
            
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
        }

        virtual void resetImplementation() override
        {
            freq_limiter_.reset();
            last_time_ = clock()->nowNanos();

            updateOutput();
        }

        virtual void update() override
        {
            DvlBase::update();

            freq_limiter_.update();

            if (freq_limiter_.isWaitComplete()) {
                updateOutput();
            }
        };

        virtual void reportState(StateReporter& reporter) override
        {
            DvlBase::reportState(reporter);
            
            // Report some shii
        }

        virtual ~DvlSimple() = default;

        const DvlSimpleParams& getParams() const
        {
            return params_;
        }

    protected:
        virtual void getBeamScans(const Pose& dvl_pose, const Pose& vehicle_pose, TTimeDelta delta_time, std::array<Vector3r, 4>& beam_unit_vecs, std::array<float, 4> &beam_ranges) = 0;

    private:
        void updateOutput()
        {
            TTimeDelta delta_time = clock()->updateSince(last_time_);

            const GroundTruth& ground_truth = getGroundTruth();

            Pose dvl_pose = params_.relative_pose + ground_truth.kinematics->pose;

            DvlData output;
            
            output.time_stamp = clock()->nowNanos();
            output.pose = dvl_pose;
            output.velocity = ground_truth.kinematics->twist.linear;
            getBeamScans(dvl_pose, ground_truth.kinematics->pose, delta_time, output.beam_unit_vecs, output.beam_ranges);

            output.altitude = 0; // TODO: cos(elevation_angle) * sum(good_angles) / num_good_beams
            output.course_gnd = atan2(output.velocity.y(), output.velocity.x());
            output.speed_gnd = sqrt(output.velocity.x() * output.velocity.x() + output.velocity.y() * output.velocity.y());

            output.num_good_beams = 4; // TODO
            output.sound_speed = 0; // TODO

            output.beam_ranges_valid = params_.return_ranges;
            output.beam_velocities_valid = true;
            
            // TODO: output.velocity_covariance = ;
            // TODO: output.range_covariance = ;

            last_time_ = output.time_stamp;

            setOutput(output);
        }

    private:
        DvlSimpleParams params_;

        FrequencyLimiter freq_limiter_;
        TTimePoint last_time_;
    };
}
}

#endif