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
            params_.initializeFromSettings(setting);

            azimuth_beam_count = params_.azimuth_angle / params_.azimuth_resolution;
            elevation_ray_count = params_.elevation_angle / params_.elevation_resolution;
            range_bin_count = (params_.range_max - params_.range_min) / params_.range_resolution;

            createLasers();

            image_.reserve(azimuth_beam_count * elevation_ray_count);

            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
        }

        // initializes information based on lidar configuration
        void createLasers()
        {
            beams.clear();

            float min_azimuth = -params_.azimuth_angle / 2;
            float min_elevation = -params_.elevation_angle / 2;
            for (int i = 0; i < azimuth_beam_count; ++i) {
                for (int j = 0; j < elevation_ray_count; ++j) {
                    const auto elevation_angle = min_elevation + ((j + 1) * params_.elevation_resolution);
                    const auto azimuth_angle = min_azimuth + ((i + 1) * params_.azimuth_resolution);

                    Quaternionr ray_q_l = VectorMath::toQuaternion(
                        Utils::degreesToRadians(elevation_angle),
                        0,
                        Utils::degreesToRadians(azimuth_angle));

                    Quaternionr ray_q_b = VectorMath::coordOrientationAdd(ray_q_l, params_.relative_pose.orientation);

                    beams.push_back(
                        VectorMath::rotateVector(VectorMath::front(),
                                                 ray_q_b,
                                                 true) *
                        params_.range_max);
                }
            }
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
            reporter.writeValue("Sonar-Num-Azimuth-Bins", azimuth_beam_count);
            reporter.writeValue("Sonar-Num-Range-Bins", range_bin_count);
            reporter.writeValue("Sonar-Num-Elevation-Rays", elevation_ray_count);
        }
        //*** End: UpdatableState implementation ***//

        virtual ~SonarSimple() = default;

        const SonarSimpleParams& getParams() const
        {
            return params_;
        }

        std::vector<Vector3r> getBeams() const
        {
            return beams;
        }

    protected:
        virtual void getPointCloud(const Pose& sonar_pose, const Pose& vehicle_pose,
                                   TTimeDelta delta_time, vector<real_T>& image,
                                   vector<real_T>& point_cloud) = 0;

    private: //methods
        void updateOutput()
        {
            TTimeDelta delta_time = clock()->updateSince(last_time_);

            image_.clear();

            const GroundTruth& ground_truth = getGroundTruth();

            Pose sonar_pose = params_.relative_pose + ground_truth.kinematics->pose;
            /*
            getPointCloud(params_.relative_pose, // relative lidar pose
                          ground_truth.kinematics->pose, // relative vehicle pose
                          delta_time,
                          image_,
                          point_cloud_
                );
            */
            SonarData output;
            output.image = image_;
            output.time_stamp = clock()->nowNanos();
            output.pose = sonar_pose;

            last_time_ = output.time_stamp;

            setOutput(output);
        }

    private:
        SonarSimpleParams params_;
        vector<real_T> image_;
        vector<real_T> point_cloud_;

        std::size_t azimuth_beam_count;
        std::size_t elevation_ray_count;
        std::size_t range_bin_count;
        std::vector<Vector3r> beams;

        FrequencyLimiter freq_limiter_;
        TTimePoint last_time_;
    };

}
} //namespace
#endif