#include "ArucoFractalModule.h"
#include <rttr/registration>

namespace traact::component::aruco {

    class ArucoFractalPoseOutput : public ArucoFractalPoseOutputComponent {
    public:
        explicit ArucoFractalPoseOutput(const std::string &name)
                : ArucoFractalPoseOutputComponent(name) {}

        static traact::pattern::Pattern::Ptr GetPattern(){
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("ArucoFractalPoseOutput", Concurrency::SERIAL, ComponentType::INTERNAL_SYNC_SOURCE);

            pattern->addProducerPort("output", spatial::Pose6DHeader::MetaType);

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            aruco_module_ = std::dynamic_pointer_cast<ArucoFractalModule>(module_);
            aruco_module_->AddOutput(this);
            return true;
        }



    };


CREATE_TRAACT_COMPONENT_FACTORY(ArucoFractalPoseOutput)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::aruco::ArucoFractalPoseOutput)
END_TRAACT_PLUGIN_REGISTRATION
