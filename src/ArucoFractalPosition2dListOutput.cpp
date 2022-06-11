#include "ArucoFractalModule.h"
#include <rttr/registration>

namespace traact::component::aruco {

    class ArucoFractalPosition2dListOutput : public ArucoFractalPoseOutputComponent {
    public:
        explicit ArucoFractalPosition2dListOutput(const std::string &name)
                : ArucoFractalPoseOutputComponent(name) {}

        static traact::pattern::Pattern::Ptr GetPattern(){
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("ArucoFractalPosition2dListOutput", Concurrency::SERIAL, ComponentType::INTERNAL_SYNC_SOURCE);

            pattern->addProducerPort("outputPoints2d", spatial::Position2DListHeader::MetaType);

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            aruco_module_ = std::dynamic_pointer_cast<ArucoFractalModule>(module_);
            aruco_module_->AddOutput(this);
            return true;
        }



    };



CREATE_TRAACT_COMPONENT_FACTORY(ArucoFractalPosition2dListOutput)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::aruco::ArucoFractalPosition2dListOutput)
END_TRAACT_PLUGIN_REGISTRATION
