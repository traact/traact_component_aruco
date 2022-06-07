#ifndef TRAACTMULTI_ARUCOFRACTALMODULE_H
#define TRAACTMULTI_ARUCOFRACTALMODULE_H

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>

#include "fractaldetector.h"
#include "aruco_cvversioning.h"


namespace traact::component::aruco {

    class ArucoFractalPoseOutputComponent;
    class ArucoFractalDebugOutputComponent;

    class ArucoFractalModule : public Module{
    public:

        bool init(ComponentPtr module_component) override;

        bool start(ComponentPtr module_component) override;

        bool stop(ComponentPtr module_component) override;

        bool teardown(ComponentPtr module_component) override;

        void AddOutput(ArucoFractalPoseOutputComponent* output_component);
        void SetDebugOutput(ArucoFractalDebugOutputComponent* debug_output_component);

        bool TrackMarker(TimestampType ts, const cv::Mat &image, const traact::vision::CameraCalibration &calibration,
                         const ::aruco::FractalMarkerSet::CONF_TYPES &marker_config, double marker_size);

        void SendNoValidInput(TimestampType ts);

    private:
        ArucoFractalPoseOutputComponent* pose_output_component_{nullptr};
        ArucoFractalDebugOutputComponent* debug_output_component_{nullptr};


    RTTR_ENABLE(Module)
    };

    class ArucoFractalComponent : public ModuleComponent {
    public:
        ArucoFractalComponent(const std::string &name, const ComponentType traact_component_type,
                       const ModuleType module_type);

        std::string GetModuleKey() override;
        Module::Ptr InstantiateModule() override;

    protected:
        std::shared_ptr<ArucoFractalModule> aruco_module_;

    RTTR_ENABLE(ModuleComponent)
    };

    class ArucoFractalPoseOutputComponent : public ArucoFractalComponent {
    public:
        explicit ArucoFractalPoseOutputComponent(const std::string &name);

        void SendMarker(spatial::Pose6DHeader::NativeType pose, TimestampType ts);

        void SendInvalid(TimestampType ts);

    RTTR_ENABLE(ArucoFractalComponent)
    };

    class ArucoFractalDebugOutputComponent : public ArucoFractalComponent {
    public:
        explicit ArucoFractalDebugOutputComponent(const std::string &name);

        void Send(cv::Mat debug_image, TimestampType ts);

    RTTR_ENABLE(ArucoFractalComponent)
    };


}




#endif //TRAACTMULTI_ARUCOFRACTALMODULE_H
