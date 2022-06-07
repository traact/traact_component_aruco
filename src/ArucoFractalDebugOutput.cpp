#include "ArucoFractalModule.h"
#include <rttr/registration>

namespace traact::component::aruco {

    class ArucoFractalDebugOutput : public ArucoFractalDebugOutputComponent {
    public:
        explicit ArucoFractalDebugOutput(const std::string &name)
                : ArucoFractalDebugOutputComponent(name) {}

        traact::pattern::Pattern::Ptr GetPattern() const override{
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("ArucoFractalDebugOutput", Concurrency::SERIAL);

            pattern->addProducerPort("output", vision::ImageHeader::MetaType);


            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            aruco_module_ = std::dynamic_pointer_cast<ArucoFractalModule>(module_);
            aruco_module_->SetDebugOutput( this);
            return true;
        }

    private:

    RTTR_ENABLE(ArucoFractalDebugOutputComponent)

    };



}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::aruco::ArucoFractalDebugOutput>("ArucoFractalDebugOutput").constructor<std::string>()();
}