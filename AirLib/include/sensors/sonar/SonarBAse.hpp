#ifndef msr_airlib_SonarBase_hpp
#define msr_airlib_SonarBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr
{
namespace airlib
{

    class SonarBase : public SensorBase
    {
    public:
        SonarBase(const std::string& sensor_name = "")
            : SensorBase(sensor_name)
        {
        }
    public:
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            UpdatableObject::reportState(reporter);

            reporter.writeValue("Sonar-Timestamp", output_.time_stamp);
            reporter.writeValue("Sonar-NumPoints", output_.image.size());
        }

        const SonarData& getOutput() const
        {
            return output_;
        }

    protected:
        void setOutput(const SonarData& output)
        {
            output_ = output;
        }

    private:
        SonarData output_;
    };
}
} //namespace
#endif