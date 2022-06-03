#include "ArucoModule.h"
#include <rttr/registration>

namespace traact::component::aruco {



    class ArucoOutput : public ArucoOutputComponent {
    public:
        explicit ArucoOutput(const std::string &name)
                : ArucoOutputComponent(name) {}

        traact::pattern::Pattern::Ptr GetPattern() const override{
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("ArucoOutput", Concurrency::serial);

            pattern->addProducerPort("output", spatial::Pose6DHeader::MetaType);
            pattern->addParameter("marker_id", 0);

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            aruco_module_ = std::dynamic_pointer_cast<ArucoModule>(module_);
            pattern::setValueFromParameter(parameter, "marker_id", marker_id_, 0);
            aruco_module_->AddOutput(marker_id_, this);
            return true;
        }

    private:
        int marker_id_{0};


    RTTR_ENABLE(Component, ModuleComponent, ArucoComponent)

    };



}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::aruco::ArucoOutput>("ArucoOutput").constructor<std::string>()();
}