
PROJECT (Firmware LANGUAGES CSharp)
CMAKE_MINIMUM_REQUIRED ( VERSION 3.12)
set(CMAKE_DOTNET_TARGET_FRAMEWORK_VERSION "4.5.2")

add_library (Firmware SHARED Firmware.cs CommandType.cs App.config Properties/AssemblyInfo.cs)

add_dependencies(Firmware Hardware)	

set(CMAKE_CSharp_FLAGS "/langversion:7") 

include(ExternalProject)
ExternalProject_Add(Hardware
PREFIX ${CMAKE_BINARY_DIR}
GIT_REPOSITORY "https://github.com/dantmw/3DPrinterSIMHardware.git"
PATCH_COMMAND ""
BINARY_DIR ${CMAKE_BINARY_DIR}/Hardware
SOURCE_DIR ${CMAKE_BINARY_DIR}/Hardware
INSTALL_DIR ${CMAKE_BINARY_DIR}/Hardware
CONFIGURE_COMMAND ""
BUILD_COMMAND ""
INSTALL_COMMAND ""
)

set_property(TARGET Firmware PROPERTY VS_DOTNET_REFERENCES "Hardware/bin/Release/Hardware.dll" )