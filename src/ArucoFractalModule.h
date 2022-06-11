#ifndef TRAACTMULTI_ARUCOFRACTALMODULE_H
#define TRAACTMULTI_ARUCOFRACTALMODULE_H

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>

#include "fractaldetector.h"
#include "aruco_cvversioning.h"


namespace traact::component::aruco {

    class ArucoFractalPoseOutputComponent;
    class ArucoFractalPosition3dListOutputComponent;
    class ArucoFractalPosition2dListOutputComponent;
    class ArucoFractalDebugOutputComponent;

    class ArucoFractalModule : public Module{
    public:

        bool init(ComponentPtr module_component) override;

        bool start(ComponentPtr module_component) override;

        bool stop(ComponentPtr module_component) override;

        bool teardown(ComponentPtr module_component) override;

        void AddOutput(ArucoFractalPoseOutputComponent* output_component);
        void AddOutput(ArucoFractalPosition3dListOutputComponent* output_component);
        void AddOutput(ArucoFractalPosition2dListOutputComponent* output_component);
        void SetDebugOutput(ArucoFractalDebugOutputComponent* debug_output_component);

        bool TrackMarker(Timestamp ts, const cv::Mat &image, const traact::vision::CameraCalibration &calibration,
                         const ::aruco::FractalMarkerSet::CONF_TYPES &marker_config, double marker_size);

        void SendNoValidInput(Timestamp ts);

    private:
        ArucoFractalPoseOutputComponent* pose_output_component_{nullptr};
        ArucoFractalPosition3dListOutputComponent* position3dlist_output_component_{nullptr};
        ArucoFractalPosition2dListOutputComponent* position2dlist_output_component_{nullptr};
        ArucoFractalDebugOutputComponent* debug_output_component_{nullptr};


    };

    class ArucoFractalComponent : public ModuleComponent {
    public:
        ArucoFractalComponent(const std::string &name, const ModuleType module_type);

        std::string getModuleKey() override;
        Module::Ptr instantiateModule() override;

    protected:
        std::shared_ptr<ArucoFractalModule> aruco_module_;


    };

    class ArucoFractalPoseOutputComponent : public ArucoFractalComponent {
    public:
        explicit ArucoFractalPoseOutputComponent(const std::string &name);

        void SendMarker(spatial::Pose6DHeader::NativeType pose, Timestamp ts);

        void SendInvalid(Timestamp ts);


    };

    class ArucoFractalPosition3dListOutputComponent : public ArucoFractalComponent {
    public:
        explicit ArucoFractalPosition3dListOutputComponent(const std::string &name);

        void SendMarker(spatial::Position3DListHeader::NativeType pose, Timestamp ts);

        void SendInvalid(Timestamp ts);


    };

    class ArucoFractalPosition2dListOutputComponent : public ArucoFractalComponent {
    public:
        explicit ArucoFractalPosition2dListOutputComponent(const std::string &name);

        void SendMarker(spatial::Position2DListHeader::NativeType pose, Timestamp ts);

        void SendInvalid(Timestamp ts);


    };

    class ArucoFractalDebugOutputComponent : public ArucoFractalComponent {
    public:
        explicit ArucoFractalDebugOutputComponent(const std::string &name);

        void Send(cv::Mat debug_image, Timestamp ts);


    };


}




#endif //TRAACTMULTI_ARUCOFRACTALMODULE_H
