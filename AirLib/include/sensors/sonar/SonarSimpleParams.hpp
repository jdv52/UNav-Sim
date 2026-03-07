#ifndef msr_airlib_SonarSimpleParams_hpp
#define msr_airlib_SonarSimpleParams_hpp

#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

    struct SonarSimpleParams
    {

        // Velodyne VLP-16 Puck config
        // https://velodynelidar.com/vlp-16.html

        // default settings

        real_T azimuth_angle = 120.0;
        real_T elevation_angle = 20.0;
        real_T range_min = 0.1;
        real_T range_max = 10.0;

        Pose relative_pose; // position and orientation

        real_T update_frequency = 10; // Hz
        real_T startup_delay = 0; // sec
    };

}
} //namespace
#endif