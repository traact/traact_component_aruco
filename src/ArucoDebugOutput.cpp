#include "ArucoModule.h"
#include <rttr/registration>

namespace traact::component::aruco {

class ArucoDebugOutput : public ArucoDebugOutputComponent {
 public:
    explicit ArucoDebugOutput(const std::string &name)
        : ArucoDebugOutputComponent(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("ArucoDebugOutput", Concurrency::SERIAL, ComponentType::INTERNAL_SYNC_SOURCE);

        pattern->addProducerPort("output", vision::ImageHeader::NativeTypeName);

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        aruco_module_ = std::dynamic_pointer_cast<ArucoModule>(module_);
        aruco_module_->SetDebugOutput(this);
        return true;
    }

 private:



};

CREATE_TRAACT_COMPONENT_FACTORY(ArucoDebugOutput)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::aruco::ArucoDebugOutput)
END_TRAACT_PLUGIN_REGISTRATION
