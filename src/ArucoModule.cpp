#include "ArucoModule.h"
#include <opencv2/aruco.hpp>
#include <traact/opencv/OpenCVUtils.h>
#include <opencv2/imgproc.hpp>
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

std::string ArucoComponent::getModuleKey() {
    return "aruco_global";
}

Module::Ptr ArucoComponent::instantiateModule() {
    return std::make_shared<ArucoModule>();
}

ArucoComponent::ArucoComponent(const std::string &name, const ComponentType traact_component_type,
                               const ModuleType module_type) : ModuleComponent(name, traact_component_type,
                                                                               module_type) {}

ArucoOutputComponent::ArucoOutputComponent(const std::string &name) : ArucoComponent(name,
                                                                                     ComponentType::INTERNAL_SYNC_SOURCE,
                                                                                     ModuleType::GLOBAL) {}

ArucoDebugOutputComponent::ArucoDebugOutputComponent(const std::string &name) : ArucoComponent(name,
                                                                                               ComponentType::INTERNAL_SYNC_SOURCE,
                                                                                               ModuleType::GLOBAL) {

}

bool ArucoModule::TrackMarker(Timestamp ts, const cv::Mat &image,
                              const traact::vision::CameraCalibration &calibration,
                              const cv::Ptr<cv::aruco::Dictionary> &dictionary,
                              const cv::Ptr<cv::aruco::DetectorParameters> &parameter, double marker_size) {

    SPDLOG_INFO("ArucoModule TrackMarker");

    std::vector<std::vector<cv::Point2f>> markers, rejected_candidates;
    std::vector<int32_t> marker_ids;
    cv::aruco::detectMarkers(
        image,
        dictionary,
        markers,
        marker_ids,
        parameter,
        rejected_candidates);

    if (debug_output_component_) {
        cv::Mat debug_image;
        cv::cvtColor(image, debug_image, cv::COLOR_GRAY2RGB);
        cv::aruco::drawDetectedMarkers(debug_image, markers, marker_ids);
        debug_output_component_->Send(debug_image, ts);
    }

    if (!marker_ids.empty()) {
        cv::Mat cameraMatrix;
        cv::Mat distortionCoefficientsMatrix;
        traact2cv(calibration, cameraMatrix, distortionCoefficientsMatrix);

        std::vector<cv::Vec3d> r_vecs;
        std::vector<cv::Vec3d> t_vecs;

        cv::aruco::estimatePoseSingleMarkers(
            markers, marker_size,
            cameraMatrix,
            distortionCoefficientsMatrix,
            r_vecs,
            t_vecs);

        for (auto &output : output_components_) {
            auto has_marker = std::find(marker_ids.begin(), marker_ids.end(), output.first);
            if (has_marker == marker_ids.end()) {
                output.second->SendInvalid(ts);
            } else {
                int marker_index = has_marker - marker_ids.begin();
                spatial::Pose6DHeader::NativeType result;
                cv2traact(r_vecs[marker_index], t_vecs[marker_index], result);
                output.second->SendMarker(result, ts);
            }

        }
    } else {
        for (auto &output : output_components_) {
            output.second->SendInvalid(ts);
        }
    }

    return true;
}

void ArucoModule::SetDebugOutput(ArucoDebugOutputComponent *debug_output_component) {
    debug_output_component_ = debug_output_component;

}

void ArucoModule::SendNoValidInput(Timestamp ts) {
    for (auto &output : output_components_) {
        output.second->SendInvalid(ts);
    }

}

void ArucoOutputComponent::SendMarker(spatial::Pose6DHeader::NativeType pose, Timestamp ts) {
    SPDLOG_INFO("ArucoOutputComponent Send {0} {1}", getName(), ts.time_since_epoch().count());
    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();
    if (buffer == nullptr) {
        SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
        return;
    }
    auto &output = buffer->getOutput<spatial::Pose6DHeader::NativeType, spatial::Pose6DHeader>(0);
    output = pose;
    buffer->commit(true);

}

void ArucoOutputComponent::SendInvalid(Timestamp ts) {
    SPDLOG_INFO("ArucoOutputComponent Invalid {0} {1}", getName(), ts.time_since_epoch().count());
    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();
    if (buffer == nullptr) {
        SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
        return;
    }
    buffer->commit(false);
}

void ArucoDebugOutputComponent::Send(cv::Mat debug_image, Timestamp ts) {
    SPDLOG_INFO("ArucoDebugOutputComponent Send {0} {1}", getName(), ts.time_since_epoch().count());
    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();
    if (buffer == nullptr) {
        SPDLOG_ERROR("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
        return;
    }
    auto &output = buffer->getOutput<vision::ImageHeader::NativeType, vision::ImageHeader>(0);
    output.SetCpuMat(debug_image);
    buffer->commit(true);
}

}