#ifndef msr_airlib_Sonar_hpp
#define msr_airlib_Sonar_hpp

#include <random>
#include "common/Common.hpp"
#include "SonarSimpleParams.hpp"
#include "SonarBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr
{
namespace airlib
{

    class SonarSimple : public SonarBase
    {
    public:
        SonarSimple(const AirSimSettings::SonarSetting& setting = AirSimSettings::SonarSetting())
            : SonarBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);

            //initialize frequency limiter
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            freq_limiter_.reset();
            last_time_ = clock()->nowNanos();

            updateOutput();
        }

        virtual void update() override
        {
            SonarBase::update();

            freq_limiter_.update();

            if (freq_limiter_.isWaitComplete()) {
                updateOutput();
            }
        }

        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            SonarBase::reportState(reporter);

            reporter.writeValue("Sonar-Azimuth-Angle", params_.azimuth_angle);
            reporter.writeValue("Sonar-Elevation-Angle", params_.elevation_angle);
            reporter.writeValue("Sonar-Range-Min", params_.range_min);
            reporter.writeValue("Sonar-Range-Max", params_.range_max);
        }
        //*** End: UpdatableState implementation ***//

        virtual ~SonarSimple() = default;

    protected:
        virtual void getPointCloud(const Pose& sonar_pose, const Pose& vehicle_pose,
                                   TTimeDelta delta_time, vector<real_T>& point_cloud) = 0;
        
        const SonarSimpleParams& getParams()
        {
            return params_;
        }

    private: //methods
        void updateOutput()
        {
            TTimeDelta delta_time = clock()->updateSince(last_time_);

            point_cloud_.clear();

            const GroundTruth& ground_truth = getGroundTruth();

            //order of Pose addition is important here because it also adds quaternions which is not commutative!
            // TODO: need to understand if there are unnecessary copies of vector being made that can be avoided.
            Pose sonar_pose = params_.relative_pose + ground_truth.kinematics->pose;
            getPointCloud(params_.relative_pose, // relative lidar pose
                          ground_truth.kinematics->pose, // relative vehicle pose
                          delta_time,
                          point_cloud_);
            SonarData output;
            output.point_cloud = point_cloud_;
            output.time_stamp = clock()->nowNanos();
            output.pose = sonar_pose;

            last_time_ = output.time_stamp;

            setOutput(output);
        }

    private:
        SonarSimpleParams params_;
        vector<real_T> point_cloud_;

        FrequencyLimiter freq_limiter_;
        TTimePoint last_time_;
    };

}
} //namespace
#endif