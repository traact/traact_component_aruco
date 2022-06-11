#ifndef TRAACTMULTI_ARUCOMODULE_H
#define TRAACTMULTI_ARUCOMODULE_H

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <opencv2/aruco.hpp>

namespace traact::component::aruco {

class ArucoOutputComponent;
class ArucoDebugOutputComponent;

class ArucoModule : public Module {
 public:

    bool init(ComponentPtr module_component) override;

    bool start(ComponentPtr module_component) override;

    bool stop(ComponentPtr module_component) override;

    bool teardown(ComponentPtr module_component) override;

    void AddOutput(int marker_id, ArucoOutputComponent *output_component);
    void SetDebugOutput(ArucoDebugOutputComponent *debug_output_component);

    bool TrackMarker(Timestamp ts, const cv::Mat &image, const traact::vision::CameraCalibration &calibration,
                     const cv::Ptr<cv::aruco::Dictionary> &dictionary,
                     const cv::Ptr<cv::aruco::DetectorParameters> &parameter, double marker_size);

    void SendNoValidInput(Timestamp ts);

 private:
    std::map<int, ArucoOutputComponent *> output_components_;
    ArucoDebugOutputComponent *debug_output_component_{nullptr};

};

class ArucoComponent : public ModuleComponent {
 public:
    ArucoComponent(const std::string &name, const ComponentType traact_component_type,
                   const ModuleType module_type);

    std::string getModuleKey() override;
    Module::Ptr instantiateModule() override;

 protected:
    std::shared_ptr<ArucoModule> aruco_module_;


};

class ArucoOutputComponent : public ArucoComponent {
 public:
    explicit ArucoOutputComponent(const std::string &name);

    void SendMarker(spatial::Pose6DHeader::NativeType pose, Timestamp ts);

    void SendInvalid(Timestamp ts);


};

class ArucoDebugOutputComponent : public ArucoComponent {
 public:
    explicit ArucoDebugOutputComponent(const std::string &name);

    void Send(cv::Mat debug_image, Timestamp ts);


};

}

#endif //TRAACTMULTI_ARUCOMODULE_H
