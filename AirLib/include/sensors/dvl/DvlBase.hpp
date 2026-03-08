#ifndef msr_airlib_DvlBase_hpp
#define msr_airlib_DvlBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr
{
namespace airlib
{

    class DvlBase : public SensorBase
    {
    public:
        DvlBase(const std::string& sensor_name = "")
            : SensorBase(sensor_name)
        {
        }

    public:
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            UpdatableObject::reportState(reporter);

            reporter.writeValue("Dvl-Timestamp", output_.time_stamp);
            reporter.writeValue("Dvl-Velocity", output_.velocity);

            std::stringstream ss;
            ss << output_.beam_ranges[0] << ", " << output_.beam_ranges[1] << ", " << output_.beam_ranges[2] << ", " << output_.beam_ranges[3];
            reporter.writeValue("Dvl-Ranges", ss.str());
        }

        const DvlData& getOutput() const
        {
            return output_;
        }

    protected:
        void setOutput(const DvlData& output)
        {
            output_ = output;
        }

    private:
        DvlData output_;
    };
}
}
#endif
