from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout


class PcaDemos(ConanFile):
    settings = "compiler", "build_type"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualBuildEnv"

    def requirements(self):
        self.requires("libhal-lpc40xx/[^1.0.0]")
        self.requires("libhal-pca/1.0.0")
        self.tool_requires("gnu-arm-embedded-toolchain/11.3.0")
        self.tool_requires("cmake-arm-embedded/0.1.1")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
