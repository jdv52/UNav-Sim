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
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            UpdatableObject::reportState(reporter);

            reporter.writeValue("Sonar-Timestamp", output_.time_stamp);
            reporter.writeValue("Sonar-NumPoints", (int)(output_.point_cloud.size() / 3));
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