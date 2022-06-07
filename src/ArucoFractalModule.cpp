#include "ArucoFractalModule.h"
#include <traact/opencv/OpenCVUtils.h>
#include <opencv2/imgproc.hpp>
#include "cvdrawingutils.h"
#include <fmt/ranges.h>

namespace traact::component::aruco {

    void ArucoFractalModule::AddOutput(ArucoFractalPoseOutputComponent *output_component) {
        SPDLOG_DEBUG("ArucoFractalModule AddPoseOutput ");
        pose_output_component_ = output_component;
    }

    bool ArucoFractalModule::init(Module::ComponentPtr module_component) {
        SPDLOG_DEBUG("ArucoFractalModule init from module_component");
        return Module::init(module_component);
    }

    bool ArucoFractalModule::start(Module::ComponentPtr module_component) {
        SPDLOG_DEBUG("ArucoFractalModule start from module_component");
        return Module::start(module_component);
    }

    bool ArucoFractalModule::stop(Module::ComponentPtr module_component) {
        SPDLOG_DEBUG("ArucoFractalModule stop from module_component");
        return Module::stop(module_component);
    }

    bool ArucoFractalModule::teardown(Module::ComponentPtr module_component) {
        SPDLOG_DEBUG("ArucoFractalModule teardown from module_component");
        return Module::teardown(module_component);
    }


    std::string ArucoFractalComponent::GetModuleKey() {
        return "aruco_fractal_global";
    }

    Module::Ptr ArucoFractalComponent::InstantiateModule() {
        return std::make_shared<ArucoFractalModule>();
    }

    ArucoFractalComponent::ArucoFractalComponent(const std::string &name, const ComponentType traact_component_type,
                                   const ModuleType module_type) : ModuleComponent(name, traact_component_type,
                                                                                   module_type) {}

    ArucoFractalPoseOutputComponent::ArucoFractalPoseOutputComponent(const std::string &name) : ArucoFractalComponent(name, ComponentType::SyncSource, ModuleType::Global) {}

    ArucoFractalPosition2dListOutputComponent::ArucoFractalPosition2dListOutputComponent(const std::string &name) : ArucoFractalComponent(name, ComponentType::SyncSource, ModuleType::Global) {}

    ArucoFractalPosition3dListOutputComponent::ArucoFractalPosition3dListOutputComponent(const std::string &name) : ArucoFractalComponent(name, ComponentType::SyncSource, ModuleType::Global) {}

    ArucoFractalDebugOutputComponent::ArucoFractalDebugOutputComponent(const std::string &name) : ArucoFractalComponent(name, ComponentType::SyncSource, ModuleType::Global) {

    }

    bool ArucoFractalModule::TrackMarker(TimestampType ts, const cv::Mat &image,
                                  const traact::vision::CameraCalibration &calibration,
                                  const ::aruco::FractalMarkerSet::CONF_TYPES &marker_config, double marker_size) {

        SPDLOG_TRACE("ArucoFractalModule TrackMarker");

        cv::Mat cameraMatrix;
        cv::Mat distortionCoefficientsMatrix;
        traact2cv(calibration, cameraMatrix, distortionCoefficientsMatrix);

        // aruco can only handle distortion coefficients with 4-7 elements, opencv requires 4/5/8, azure kinect provides 8
        // for now only take 5 (3 radial, 2 tangential) parameters
        auto dcm = distortionCoefficientsMatrix(cv::Range(0,5),cv::Range(0,1));

        ::aruco::CameraParameters CamParam;
        CamParam.setParams(cameraMatrix, dcm, cv::Size(calibration.width, calibration.height));

        ::aruco::FractalDetector FractalDetector;
        FractalDetector.setConfiguration(marker_config);

        if (CamParam.isValid())
        {
            CamParam.resize(image.size());
            FractalDetector.setParams(CamParam, static_cast<float>(marker_size));
        }

        std::vector<::aruco::Marker> detected_markers;
        std::vector<int> marker_ids;

        FractalDetector.detect(image);
        cv::Mat rvec, tvec;
        std::vector<cv::Point3f> points3d;
        std::vector<cv::Point2f> points2d;
        bool found_marker{false};

        if (FractalDetector.poseEstimation()) {
            if (pose_output_component_) {
                rvec = FractalDetector.getRvec();
                tvec = FractalDetector.getTvec();
            }
            if (position2dlist_output_component_) {
                points2d = FractalDetector.getPoints2d(image);
            }
            if (position3dlist_output_component_) {
                points3d = FractalDetector.getPoints3d(image);
            }
            found_marker = true;
        }

        // pose could be detected
        if(debug_output_component_){
            cv::Mat debug_image;
            cv::cvtColor(image, debug_image, cv::COLOR_GRAY2RGB);
            if (found_marker) {
                FractalDetector.draw3d(debug_image);
            } else {
                FractalDetector.draw2d(debug_image);
            }
            debug_output_component_->Send(debug_image,ts);
        }


        if (found_marker) {
            if (pose_output_component_) {
                spatial::Pose6DHeader::NativeType result;
                cv2traact(rvec, tvec, result);
                pose_output_component_->SendMarker(result, ts);
            }
            if (position3dlist_output_component_) {
                spatial::Position3DListHeader::NativeType result;
                // should probably be in cv2traact
                for (auto& p: points3d) {
                    spatial::Position3DListHeader::NativeType::value_type v;
                    v(0) = p.x;
                    v(1) = p.y;
                    v(2) = p.z;
                    result.push_back(v);
                }
                position3dlist_output_component_->SendMarker(result, ts);
            }
            if (position2dlist_output_component_) {
                spatial::Position2DListHeader::NativeType result;
                // should probably be in cv2traact
                for (auto& p: points2d) {
                    spatial::Position2DListHeader::NativeType::value_type v;
                    v(0) = p.x;
                    v(1) = p.y;
                    result.push_back(v);
                }
                position2dlist_output_component_->SendMarker(result, ts);
            }

        } else {
            if (pose_output_component_) {
                pose_output_component_->SendInvalid(ts);
            }
            if (position3dlist_output_component_) {
                position3dlist_output_component_->SendInvalid(ts);
            }
            if (position2dlist_output_component_) {
                position2dlist_output_component_->SendInvalid(ts);
            }
        }

        return true;
    }

    void ArucoFractalModule::SetDebugOutput(ArucoFractalDebugOutputComponent *debug_output_component) {
        debug_output_component_ = debug_output_component;
    }

    void ArucoFractalModule::SendNoValidInput(TimestampType ts) {
        if (pose_output_component_) {
            pose_output_component_->SendInvalid(ts);
        }
    }

    void ArucoFractalPoseOutputComponent::SendMarker(spatial::Pose6DHeader::NativeType pose, TimestampType ts) {
        SPDLOG_DEBUG("ArucoFractalPoseOutputComponent Send {0} {1}",getName(), ts.time_since_epoch().count());
        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer == nullptr){
            SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }
        auto& output = buffer->getOutput<spatial::Pose6DHeader::NativeType, spatial::Pose6DHeader>(0);
        output = pose;
        buffer->Commit(true);
    }

    void ArucoFractalPoseOutputComponent::SendInvalid(TimestampType ts) {
        SPDLOG_DEBUG("ArucoFractalPoseOutputComponent Invalid {0} {1}",getName(), ts.time_since_epoch().count());
        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer == nullptr){
            SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }
        buffer->Commit(false);
    }

    void ArucoFractalPosition3dListOutputComponent::SendMarker(spatial::Position3DListHeader::NativeType points3d, TimestampType ts) {
        SPDLOG_DEBUG("ArucoFractalPosition3dListOutputComponent Send {0} {1}",getName(), ts.time_since_epoch().count());
        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer == nullptr){
            SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }
        auto& output = buffer->getOutput<spatial::Position3DListHeader::NativeType, spatial::Position3DListHeader>(0);
        output = points3d;
        buffer->Commit(true);
    }

    void ArucoFractalPosition3dListOutputComponent::SendInvalid(TimestampType ts) {
        SPDLOG_DEBUG("ArucoFractalPosition3dListOutputComponent Invalid {0} {1}",getName(), ts.time_since_epoch().count());
        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer == nullptr){
            SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }
        buffer->Commit(false);
    }

    void ArucoFractalPosition2dListOutputComponent::SendMarker(spatial::Position2DListHeader::NativeType points2d, TimestampType ts) {
        SPDLOG_DEBUG("ArucoFractalPosition2dListOutputComponent Send {0} {1}",getName(), ts.time_since_epoch().count());
        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer == nullptr){
            SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }
        auto& output = buffer->getOutput<spatial::Position2DListHeader::NativeType, spatial::Position2DListHeader>(0);
        output = points2d;
        buffer->Commit(true);
    }

    void ArucoFractalPosition2dListOutputComponent::SendInvalid(TimestampType ts) {
        SPDLOG_DEBUG("ArucoFractalPosition2dListOutputComponent Invalid {0} {1}",getName(), ts.time_since_epoch().count());
        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer == nullptr){
            SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }
        buffer->Commit(false);
    }

    void ArucoFractalDebugOutputComponent::Send(cv::Mat debug_image, TimestampType ts) {
        SPDLOG_DEBUG("ArucoFractalDebugOutputComponent Send {0} {1}",getName(), ts.time_since_epoch().count());
        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer == nullptr){
            SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }
        auto& output = buffer->getOutput<vision::ImageHeader::NativeType, vision::ImageHeader>(0);
        output.SetCpuMat(debug_image);
        buffer->Commit(true);
    }


}