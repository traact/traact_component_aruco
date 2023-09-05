# /usr/bin/python3
import os
from conan import ConanFile
from conan.tools.build import can_run

class TraactPackage(ConanFile):
    python_requires = "traact_base/0.0.0@traact/latest"
    python_requires_extend = "traact_base.TraactPackageCmake"

    name = "traact_component_aruco"
    version = "0.0.0"
    description = "Vision components using the aruco library"
    url = "https://github.com/traact/traact_component_aruco.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    exports_sources = "src/*", "CMakeLists.txt"

    options = {
        "shared": [True, False],
        "trace_logs_in_release": [True, False],
        "with_cuda": [True, False]
    }

    default_options = {
        "shared": True,
        "trace_logs_in_release": True,
        "with_cuda" : True,
        "opencv/*:with_jpeg": "libjpeg-turbo",
        "opencv/*:with_quirc": False,
        "libtiff/*:jpeg": "libjpeg-turbo"
    }

    def requirements(self):        
        self.requires("traact_spatial/0.0.0@traact/latest")
        self.requires("traact_vision/0.0.0@traact/latest")        
        self.requires("opencv/4.8.0@camposs/stable", override=True)
        #self.requires("libwebp/1.3.1", override=True)
        self.requires("aruco/3.1.15@camposs/stable")        

    def configure(self):
        self.options['opencv'].shared = self.options.shared
        self.options['opencv'].with_cuda = self.options.with_cuda
        # self.options['opencv'].with_tbb = True
        if self.settings.os == "Linux":            
            self.options['opencv/*'].with_gtk = True 

    def _after_package_info(self):
        self.cpp_info.libs = ["traact_component_aruco"]