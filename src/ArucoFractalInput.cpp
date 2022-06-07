#include "ArucoFractalModule.h"
#include <rttr/registration>
#include <traact/pattern/Pattern.h>

namespace traact::component::aruco {

    class ArucoFractalInput : public ArucoFractalComponent {
    public:
        ArucoFractalInput(const std::string &name)
                : ArucoFractalComponent(name, ComponentType::SYNC_SINK, ModuleType::GLOBAL) {}

        traact::pattern::Pattern::Ptr GetPattern() const {
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("ArucoFractalInput", Concurrency::SERIAL);

            pattern->addConsumerPort("input", ImageHeader::MetaType);
            pattern->addConsumerPort("input_calibration", CameraCalibrationHeader::MetaType);
            pattern->addParameter("MarkerConfig","FRACTAL_2L_6",
                                  {"FRACTAL_2L_6", "FRACTAL_3L_6", "FRACTAL_4L_6", "FRACTAL_5L_6",})
            .addParameter("MarkerSize", 0.08);
            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            aruco_module_ = std::dynamic_pointer_cast<ArucoFractalModule>(module_);

            ::aruco::FractalMarkerSet::CONF_TYPES config;
            pattern::setValueFromParameter(parameter,"MarkerConfig", config, "FRACTAL_2L_6",
                                           {
                                                   {"FRACTAL_2L_6", ::aruco::FractalMarkerSet::CONF_TYPES::FRACTAL_2L_6},
                                                   {"FRACTAL_3L_6", ::aruco::FractalMarkerSet::CONF_TYPES::FRACTAL_3L_6},
                                                   {"FRACTAL_4L_6", ::aruco::FractalMarkerSet::CONF_TYPES::FRACTAL_4L_6},
                                                   {"FRACTAL_5L_6", ::aruco::FractalMarkerSet::CONF_TYPES::FRACTAL_5L_6}
                                           });


            pattern::setValueFromParameter(parameter,"MarkerSize",marker_size_, 0.10);

            marker_config_ = config;
            return true;
        }

        bool processTimePoint(traact::DefaultComponentBuffer &data) override {
            using namespace traact::vision;
            const auto& input_image = data.getInput<ImageHeader>(0).GetCpuMat();
            const auto& input_calibration = data.getInput<CameraCalibrationHeader>(1);

            return aruco_module_->TrackMarker(data.getTimestamp(), input_image, input_calibration, marker_config_, marker_size_);
        }

        // crucial in this module component as all outputs are independent sources from the dataflows point of view, so if no input is available then all outputs must send invalid
        void invalidTimePoint(Timestamp ts, std::size_t mea_idx) override {
            aruco_module_->SendNoValidInput(ts);
        }


    private:
        ::aruco::FractalMarkerSet::CONF_TYPES marker_config_;
        double marker_size_;

    RTTR_ENABLE(Component, ModuleComponent,ArucoFractalComponent)

    };

}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::aruco::ArucoFractalInput>("ArucoFractalInput").constructor<std::string>()();
}