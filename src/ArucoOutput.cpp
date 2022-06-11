#include "ArucoModule.h"
#include <rttr/registration>

namespace traact::component::aruco {

class ArucoOutput : public ArucoOutputComponent {
 public:
    explicit ArucoOutput(const std::string &name)
        : ArucoOutputComponent(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("ArucoOutput", Concurrency::SERIAL, ComponentType::INTERNAL_SYNC_SOURCE);

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



};

CREATE_TRAACT_COMPONENT_FACTORY(ArucoOutput)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::aruco::ArucoOutput)
END_TRAACT_PLUGIN_REGISTRATION
