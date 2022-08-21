/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/vision.h>
#include <traact/spatial.h>

#include <fractaldetector.h>
#include <aruco_cvversioning.h>
#include <traact/opencv/OpenCVUtils.h>

namespace traact::component::aruco {

class ArucoFractalTracker : public Component {
 public:
    using InPortImage = traact::buffer::PortConfig<vision::ImageHeader, 0>;
    using InPortCalibration = traact::buffer::PortConfig<vision::CameraCalibrationHeader, 1>;

    using OutPortPose = traact::buffer::PortConfig<spatial::Pose6DHeader, 0>;
    using OutPortPosition2D = traact::buffer::PortConfig<vision::Position2DListHeader, 1>;
    using OutPortPosition3D = traact::buffer::PortConfig<vision::Position3DListHeader, 2>;
    using OutPortDebugImage = traact::buffer::PortConfig<vision::ImageHeader, 3>;

    ArucoFractalTracker(const std::string &name)
        : Component(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("ArucoFractalTracker",
                                                       Concurrency::UNLIMITED,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addConsumerPort<InPortCalibration>("input_calibration")
            .addProducerPort<OutPortPose>("output")
            .addProducerPort<OutPortPosition2D>("output_position2D")
            .addProducerPort<OutPortPosition3D>("output_position3D")
            .addProducerPort<OutPortDebugImage>("output_debug_image")
            .addParameter("MarkerConfig", "FRACTAL_2L_6",
                          {"FRACTAL_2L_6", "FRACTAL_3L_6", "FRACTAL_4L_6", "FRACTAL_5L_6",})
            .addParameter("marker_size", 0.08);

        return pattern;
    }

    void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        connected_output_ports_ = pattern_instance.getOutputPortsConnected(kDefaultTimeDomain);
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {

        ::aruco::FractalMarkerSet::CONF_TYPES config;
        pattern::setValueFromParameter(pattern_instance, "MarkerConfig", config, "FRACTAL_2L_6",
                                       {
                                           {"FRACTAL_2L_6", ::aruco::FractalMarkerSet::CONF_TYPES::FRACTAL_2L_6},
                                           {"FRACTAL_3L_6", ::aruco::FractalMarkerSet::CONF_TYPES::FRACTAL_3L_6},
                                           {"FRACTAL_4L_6", ::aruco::FractalMarkerSet::CONF_TYPES::FRACTAL_4L_6},
                                           {"FRACTAL_5L_6", ::aruco::FractalMarkerSet::CONF_TYPES::FRACTAL_5L_6}
                                       });

        pattern::setValueFromParameter(pattern_instance, "marker_size", marker_size_, 0.10);

        marker_config_ = config;
        return true;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto input_image = data.getInput<InPortImage>().value();
        const auto &input_calibration = data.getInput<InPortCalibration>();

        SPDLOG_TRACE("ArucoFractalModule TrackMarker");

        cv::Mat cameraMatrix;
        cv::Mat distortionCoefficientsMatrix;
        traact2cv(input_calibration, cameraMatrix, distortionCoefficientsMatrix);

        auto dcm = distortionCoefficientsMatrix;
        // aruco can only handle distortion coefficients with 4-7 elements, opencv requires 4/5/8, azure kinect provides 8
        // for now only take 5 (3 radial, 2 tangential) parameters
        if (input_calibration.radial_distortion.size() > 3)
            dcm = distortionCoefficientsMatrix(cv::Range(0, 5), cv::Range(0, 1));

        ::aruco::CameraParameters CamParam;
        CamParam.setParams(cameraMatrix, dcm, cv::Size(input_calibration.width, input_calibration.height));

        ::aruco::FractalDetector FractalDetector;
        FractalDetector.setConfiguration(marker_config_);

        if (CamParam.isValid()) {
            CamParam.resize(input_image.size());
            FractalDetector.setParams(CamParam, static_cast<float>(marker_size_));
        }

        std::vector<::aruco::Marker> detected_markers;
        std::vector<int> marker_ids;

        FractalDetector.detect(input_image);
        bool found_marker{false};

        if (FractalDetector.poseEstimation()) {
            if (connected_output_ports_[OutPortPose::PortIdx]) {
                auto r_vec = FractalDetector.getRvec();
                auto t_vec = FractalDetector.getTvec();
                auto &output = data.getOutput<OutPortPose>();
                cv2traact(r_vec, t_vec, output);
            }
            if (connected_output_ports_[OutPortPosition2D::PortIdx]) {
                auto &output = data.getOutput<OutPortPosition2D>();
                output = FractalDetector.getPoints2d(input_image);
            }
            if (connected_output_ports_[OutPortPosition3D::PortIdx]) {
                auto &output = data.getOutput<OutPortPosition3D>();
                output = FractalDetector.getPoints3d(input_image);
            }
            found_marker = true;
        }

        // pose could be detected
        if (connected_output_ports_[OutPortDebugImage::PortIdx]) {
            auto &debug_image = data.getOutput<OutPortDebugImage>().value();
            auto& debug_image_header = data.getOutputHeader<OutPortDebugImage>();
            debug_image_header.width = input_image.cols;
            debug_image_header.height = input_image.rows;
            debug_image_header.pixel_format = PixelFormat::RGB;
            debug_image_header.base_type = BaseType::UINT_8;
            debug_image_header.channels = 3;
            debug_image_header.stride = debug_image_header.width;
            cv::cvtColor(input_image, debug_image, cv::COLOR_GRAY2RGB);

            if (found_marker) {
                FractalDetector.draw3d(debug_image);
            } else {
                FractalDetector.draw2d(debug_image);
            }
        }

        return true;
    }


 private:
    ::aruco::FractalMarkerSet::CONF_TYPES marker_config_;
    double marker_size_;
    pattern::instance::LocalConnectedOutputPorts connected_output_ports_;

};

CREATE_TRAACT_COMPONENT_FACTORY(ArucoFractalTracker)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::aruco::ArucoFractalTracker)
END_TRAACT_PLUGIN_REGISTRATION

