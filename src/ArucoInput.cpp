/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#include "ArucoModule.h"
#include <rttr/registration>
#include <traact/pattern/Pattern.h>
namespace traact::component::aruco {

class ArucoInput : public ArucoComponent {
 public:
    ArucoInput(const std::string &name)
        : ArucoComponent(name, ComponentType::SYNC_SINK, ModuleType::GLOBAL) {}

    traact::pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("ArucoInput", Concurrency::SERIAL);

        pattern->addConsumerPort("input", ImageHeader::MetaType);
        pattern->addConsumerPort("input_calibration", CameraCalibrationHeader::MetaType);
        pattern->addParameter("Dictionary", "DICT_4X4_50", {"DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50"})
            .addParameter("MarkerSize", 0.08);
        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        aruco_module_ = std::dynamic_pointer_cast<ArucoModule>(module_);
        cv::aruco::PREDEFINED_DICTIONARY_NAME dict;
        pattern::setValueFromParameter(parameter,
                                       "Dictionary",
                                       dict,
                                       "DICT_4X4_50",
                                       {{"DICT_4X4_50", cv::aruco::DICT_4X4_50},
                                        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
                                        {"DICT_6X6_50", cv::aruco::DICT_6X6_50}});
        pattern::setValueFromParameter(parameter, "MarkerSize", marker_size_, 0.08);

        dictionary_ = cv::aruco::getPredefinedDictionary(dict);

        parameter_ = cv::aruco::DetectorParameters::create();

        return true;
    }

    bool processTimePoint(traact::DefaultComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &input_image = data.getInput<ImageHeader>(0).GetCpuMat();
        const auto &input_calibration = data.getInput<CameraCalibrationHeader>(1);

        return aruco_module_->TrackMarker(data.getTimestamp(), input_image, input_calibration, dictionary_,
                                          parameter_, marker_size_);
    }

    // crucial in this module component as all outputs are independent sources from the dataflows point of view, so if no input is available then all outputs must send invalid
    void invalidTimePoint(Timestamp ts, size_t mea_idx) override {
        aruco_module_->SendNoValidInput(ts);
    }

 private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameter_;
    double marker_size_;

 RTTR_ENABLE(Component, ModuleComponent, ArucoComponent)

};

}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::aruco::ArucoInput>("ArucoInput").constructor<std::string>()();
}