/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include "ArucoModule.h"
#include <rttr/registration>
#include <traact/pattern/Pattern.h>
namespace traact::component::aruco {



    class ArucoInput : public ArucoComponent {
    public:
        ArucoInput(const std::string &name)
                : ArucoComponent(name, ComponentType::SyncSink, ModuleType::Global) {}

        traact::pattern::Pattern::Ptr GetPattern() const {
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("ArucoInput", Concurrency::serial);

            pattern->addConsumerPort("input", ImageHeader::MetaType);
            pattern->addConsumerPort("input_calibration", CameraCalibrationHeader::MetaType);
            pattern->addParameter("Dictionary","ARUCO_MIP_36h12",
                                  {"ARUCO_MIP_36h12", "ARUCO", "ARUCO_MIP_25h7", "ARUCO_MIP_16h3",
                                   "ARTAG", "ARTOOLKITPLUS", "ARTOOLKITPLUSBCH",
                                   "TAG16h5", "TAG25h7", "TAG25h9", "TAG36h11", "TAG36h10"})
            .addParameter("MarkerSize", 0.08);
            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            aruco_module_ = std::dynamic_pointer_cast<ArucoModule>(module_);
            ::aruco::Dictionary::DICT_TYPES dict;
            pattern::setValueFromParameter(parameter,"Dictionary", dict, "ARUCO_MIP_36h12",
                                           {
                                                {"ARUCO_MIP_36h12", ::aruco::Dictionary::DICT_TYPES::ARUCO_MIP_36h12},
                                                {"ARUCO", ::aruco::Dictionary::DICT_TYPES::ARUCO},
                                                {"ARUCO_MIP_25h7", ::aruco::Dictionary::DICT_TYPES::ARUCO_MIP_25h7},
                                                {"ARUCO_MIP_16h3", ::aruco::Dictionary::DICT_TYPES::ARUCO_MIP_16h3},
                                                {"ARTAG", ::aruco::Dictionary::DICT_TYPES::ARTAG},
                                                {"ARTOOLKITPLUS", ::aruco::Dictionary::DICT_TYPES::ARTOOLKITPLUS},
                                                {"ARTOOLKITPLUSBCH", ::aruco::Dictionary::DICT_TYPES::ARTOOLKITPLUSBCH},
                                                {"TAG16h5", ::aruco::Dictionary::DICT_TYPES::TAG16h5},
                                                {"TAG25h7", ::aruco::Dictionary::DICT_TYPES::TAG25h7},
                                                {"TAG25h9", ::aruco::Dictionary::DICT_TYPES::TAG25h9},
                                                {"TAG36h11", ::aruco::Dictionary::DICT_TYPES::TAG36h11},
                                                {"TAG36h10", ::aruco::Dictionary::DICT_TYPES::TAG36h10}
                                           });
            pattern::setValueFromParameter(parameter,"MarkerSize",marker_size_, 0.10);

            dictionary_ = dict;
            return true;
        }

        bool processTimePoint(traact::DefaultComponentBuffer &data) override {
            using namespace traact::vision;
            const auto& input_image = data.getInput<ImageHeader::NativeType, ImageHeader>(0).GetCpuMat();
            const auto& input_calibration = data.getInput<CameraCalibrationHeader::NativeType, CameraCalibrationHeader>(1);

            return aruco_module_->TrackMarker(data.GetTimestamp(), input_image, input_calibration, dictionary_, marker_size_);
        }

        // crucial in this module component as all outputs are independent sources from the dataflows point of view, so if no input is available then all outputs must send invalid
        void invalidTimePoint(TimestampType ts, std::size_t mea_idx) override {
            aruco_module_->SendNoValidInput(ts);
        }


    private:
        ::aruco::Dictionary::DICT_TYPES dictionary_;
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