# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class TraactPackage(ConanFile):
    python_requires = "traact_run_env/1.0.0@traact/latest"
    python_requires_extend = "traact_run_env.TraactPackageCmake"

    name = "traact_component_aruco"
    description = "Vision components using the aruco library"
    url = "https://github.com/traact/traact_component_aruco.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    exports_sources = "src/*", "CMakeLists.txt"

    def requirements(self):
        if self.options.with_tests:
            self.requires("gtest/[>=1.11.0]")
        self.traact_requires("traact_spatial", "latest")
        self.traact_requires("traact_vision", "latest")
        self.requires("aruco/3.1.15@camposs/stable")