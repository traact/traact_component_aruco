#include "ArucoModule.h"
#include <traact/opencv/OpenCVUtils.h>
#include <opencv2/imgproc.hpp>
#include "cvdrawingutils.h"
#include <fmt/ranges.h>

namespace traact::component::aruco {



    void ArucoModule::AddOutput(int marker_id, ArucoOutputComponent *output_component) {
        SPDLOG_INFO("ArucoModule AddOutput marker_id {0}", marker_id);
        output_components_.emplace(marker_id, output_component);

    }

    bool ArucoModule::init(Module::ComponentPtr module_component) {
        SPDLOG_INFO("ArucoModule init from module_component");
        return Module::init(module_component);
    }

    bool ArucoModule::start(Module::ComponentPtr module_component) {
        SPDLOG_INFO("ArucoModule start from module_component");
        return Module::start(module_component);
    }

    bool ArucoModule::stop(Module::ComponentPtr module_component) {
        SPDLOG_INFO("ArucoModule stop from module_component");
        return Module::stop(module_component);
    }

    bool ArucoModule::teardown(Module::ComponentPtr module_component) {
        SPDLOG_INFO("ArucoModule teardown from module_component");
        return Module::teardown(module_component);
    }


    std::string ArucoComponent::GetModuleKey() {
        return "aruco_global";
    }

    Module::Ptr ArucoComponent::InstantiateModule() {
        return std::make_shared<ArucoModule>();
    }

    ArucoComponent::ArucoComponent(const std::string &name, const ComponentType traact_component_type,
                                   const ModuleType module_type) : ModuleComponent(name, traact_component_type,
                                                                                   module_type) {}

    ArucoOutputComponent::ArucoOutputComponent(const std::string &name) : ArucoComponent(name, ComponentType::SyncSource, ModuleType::Global) {}

    ArucoDebugOutputComponent::ArucoDebugOutputComponent(const std::string &name) : ArucoComponent(name, ComponentType::SyncSource, ModuleType::Global) {

    }



    bool ArucoModule::TrackMarker(TimestampType ts, const cv::Mat &image,
                                  const traact::vision::CameraCalibration &calibration,
                                  const ::aruco::Dictionary::DICT_TYPES &dictionary, double marker_size) {

        SPDLOG_INFO("ArucoModule TrackMarker");

        cv::Mat cameraMatrix;
        cv::Mat distortionCoefficientsMatrix;
        traact2cv(calibration, cameraMatrix, distortionCoefficientsMatrix);

        // aruco can only handle distortion coefficients with 4-7 elements, opencv requires 4/5/8, azure kinect provides 8
        // for now only take 5 (3 radial, 2 tangential) parameters
        auto dcm = distortionCoefficientsMatrix(cv::Range(0,5),cv::Range(0,1));

        ::aruco::CameraParameters CamParam;
        CamParam.setParams(cameraMatrix, dcm, cv::Size(calibration.width, calibration.height));

        ::aruco::MarkerDetector MarkerDetector;
        MarkerDetector.setDictionary(dictionary, 0.);

        if (CamParam.isValid())
        {
            CamParam.resize(image.size());
        }

        std::vector<::aruco::Marker> detected_markers;
        std::map<uint32_t, ::aruco::MarkerPoseTracker> MTracker;
        std::vector<int> marker_ids;

        detected_markers = MarkerDetector.detect(image, CamParam, marker_size);

        for (auto & marker: detected_markers) {
            MTracker[marker.id].estimatePose(marker, CamParam, marker_size);
            marker_ids.push_back(marker.id);
        }

        if(debug_output_component_){
            cv::Mat debug_image;
            cv::cvtColor(image, debug_image, cv::COLOR_GRAY2RGB);

            for (auto & marker : detected_markers)
            {
                marker.draw(debug_image, cv::Scalar(0, 0, 255),2,true);
                if (marker.isPoseValid()) {
                    ::aruco::CvDrawingUtils::draw3dCube(debug_image, marker, CamParam);
                }
            }
            debug_output_component_->Send(debug_image,ts);
        }

        if (!detected_markers.empty())
        {
            for (auto& output : output_components_) {
                auto has_marker = std::find(marker_ids.begin(), marker_ids.end(), output.first);
                if(has_marker == marker_ids.end()){
                    output.second->SendInvalid(ts);
                } else {
                    int marker_index = has_marker - marker_ids.begin();
                    auto& marker = detected_markers[marker_index];
                    if (marker.isPoseValid()) {
                        spatial::Pose6DHeader::NativeType result;
                        auto rvec = MTracker[marker.id].getRvec();
                        auto tvec = MTracker[marker.id].getTvec();
                        cv2traact(rvec, tvec, result);
                        output.second->SendMarker(result, ts);
                    } else {
                        output.second->SendInvalid(ts);
                    }
                }
            }
        } else {
            for (auto& output : output_components_) {
                output.second->SendInvalid(ts);
            }
        }

        return true;
    }

    void ArucoModule::SetDebugOutput(ArucoDebugOutputComponent *debug_output_component) {
        debug_output_component_ = debug_output_component;

    }

    void ArucoModule::SendNoValidInput(TimestampType ts) {
        for (auto& output : output_components_) {
            output.second->SendInvalid(ts);
        }

    }

    void ArucoOutputComponent::SendMarker(spatial::Pose6DHeader::NativeType pose, TimestampType ts) {
        SPDLOG_INFO("ArucoOutputComponent Send {0} {1}",getName(), ts.time_since_epoch().count());
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

    void ArucoOutputComponent::SendInvalid(TimestampType ts) {
        SPDLOG_INFO("ArucoOutputComponent Invalid {0} {1}",getName(), ts.time_since_epoch().count());
        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer == nullptr){
            SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }
        buffer->Commit(false);
    }

    void ArucoDebugOutputComponent::Send(cv::Mat debug_image, TimestampType ts) {
        SPDLOG_INFO("ArucoDebugOutputComponent Send {0} {1}",getName(), ts.time_since_epoch().count());
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