#include "ArucoFractalModule.h"
#include <rttr/registration>

namespace traact::component::aruco {

    class ArucoFractalDebugOutput : public ArucoFractalDebugOutputComponent {
    public:
        explicit ArucoFractalDebugOutput(const std::string &name)
                : ArucoFractalDebugOutputComponent(name) {}

        static traact::pattern::Pattern::Ptr GetPattern(){
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("ArucoFractalDebugOutput", Concurrency::SERIAL,ComponentType::INTERNAL_SYNC_SOURCE);

            pattern->addProducerPort("output", vision::ImageHeader::MetaType);


            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            aruco_module_ = std::dynamic_pointer_cast<ArucoFractalModule>(module_);
            aruco_module_->SetDebugOutput( this);
            return true;
        }

    private:



    };



CREATE_TRAACT_COMPONENT_FACTORY(ArucoFractalDebugOutput)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::aruco::ArucoFractalDebugOutput)
END_TRAACT_PLUGIN_REGISTRATION
