cmake_minimum_required(VERSION 3.8)
project (KalmanEigen)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

set(Upstrea_VERSION 0.0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) #útil pra YCM (youcompleteme)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

if(EIGEN3_FOUND)
message (WARNING "EIGEN3 ENCONTRADO")
message (WARNING "PATH EM ${Eigen3_DIR}")
endif()

include(GenerateExportHeader)


add_library(KalmanLIB STATIC src/Kalman.cpp)
target_include_directories(KalmanLIB PUBLIC include)
target_link_libraries(KalmanLIB Eigen3::Eigen)
generate_export_header(KalmanLIB)

set_property(TARGET KalmanLIB PROPERTY VERSION ${Upstream_VERSION})
set_property(TARGET KalmanLIB PROPERTY SOVERSION 0)
set_property(TARGET KalmanLIB  PROPERTY
	INTERFACE_KalmanLIB_MAJOR_VERSION 0)
set_property(TARGET KalmanLIB APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING KalmanLIB_MAJOR_VERSION
)

install(TARGETS KalmanLIB EXPORT KalmanLibTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  FILES
    Kalman.h
    "${CMAKE_CURRENT_BINARY_DIR}/kalmanlib_export.h"
  DESTINATION
    include
  COMPONENT
    Devel
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/KalmanLIB/KalmanLIBConfigVersion.cmake"
  VERSION ${Upstream_VERSION}
  COMPATIBILITY AnyNewerVersion
)


export(EXPORT KalmanLibTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/KalmanLIB/KalmanLIBTargets.cmake"
  NAMESPACE Upstream::
)

configure_file(cmake/KalmanLIBConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/KalmanLIB/KalmanLIBStatsConfig.cmake"
  COPYONLY
)


set(ConfigPackageLocation lib/cmake/KalmanLIB)
install(EXPORT KalmanLIBTargets
  FILE
    KalmanLIBTargets.cmake
  NAMESPACE
    Upstream::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/KalmanLIBConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/KalmanLIB/KalmanLIBConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
