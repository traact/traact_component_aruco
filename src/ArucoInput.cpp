/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "ArucoModule.h"


namespace traact::component::aruco {

class ArucoInput : public ArucoComponent {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using InPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    ArucoInput(const std::string &name)
        : ArucoComponent(name, ComponentType::SYNC_SINK, ModuleType::GLOBAL) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("ArucoInput", Concurrency::SERIAL, ComponentType::SYNC_SINK);

        pattern->addConsumerPort<InPortImage>("input")
            .addConsumerPort<InPortCalibration>("input_calibration")
            .addParameter("Dictionary", "DICT_4X4_50", {"DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50"})
            .addParameter("marker_size", 0.08);
        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        aruco_module_ = std::dynamic_pointer_cast<ArucoModule>(module_);
        cv::aruco::PREDEFINED_DICTIONARY_NAME dict;

        pattern::setValueFromParameter(pattern_instance,
                                       "Dictionary",
                                       dict,
                                       "DICT_4X4_50",
                                       {{"DICT_4X4_50", cv::aruco::DICT_4X4_50},
                                        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
                                        {"DICT_6X6_50", cv::aruco::DICT_6X6_50}});
        pattern::setValueFromParameter(pattern_instance, "marker_size", marker_size_, 0.08);

        dictionary_ = cv::aruco::getPredefinedDictionary(dict);

        parameter_ = cv::aruco::DetectorParameters::create();

        return true;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &input_image = data.getInput<InPortImage>().value();
        const auto &input_calibration = data.getInput<InPortCalibration>();

        return aruco_module_->TrackMarker(data.getTimestamp(), input_image, input_calibration, dictionary_,
                                          parameter_, marker_size_);
    }

    // crucial in this module component as all outputs are independent sources from the dataflows point of view, so if no input is available then all outputs must send invalid
    bool processTimePointWithInvalid(buffer::ComponentBuffer &data) override {
        aruco_module_->SendNoValidInput(data.getTimestamp());
    }

 private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameter_;
    double marker_size_;

};


CREATE_TRAACT_COMPONENT_FACTORY(ArucoInput)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::aruco::ArucoInput)
END_TRAACT_PLUGIN_REGISTRATION
