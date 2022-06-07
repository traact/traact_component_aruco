#include "ArucoFractalModule.h"
#include <rttr/registration>

namespace traact::component::aruco {

    class ArucoFractalPosition2dListOutput : public ArucoFractalPoseOutputComponent {
    public:
        explicit ArucoFractalPosition2dListOutput(const std::string &name)
                : ArucoFractalPoseOutputComponent(name) {}

        traact::pattern::Pattern::Ptr GetPattern() const override{
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("ArucoFractalPosition2dListOutput", Concurrency::SERIAL);

            pattern->addProducerPort("outputPoints2d", spatial::Position2DListHeader::MetaType);

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            aruco_module_ = std::dynamic_pointer_cast<ArucoFractalModule>(module_);
            aruco_module_->AddOutput(this);
            return true;
        }

        RTTR_ENABLE(Component, ModuleComponent,ArucoFractalPoseOutputComponent)

    };



}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::aruco::ArucoFractalPosition2dListOutput>("ArucoFractalPosition2dListOutput").constructor<std::string>()();
}