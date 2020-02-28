cmake_minimum_required(VERSION 3.8)
project (KalmanEigen)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) #útil pra YCM (youcompleteme)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

if(EIGEN3_FOUND)
message (WARNING "EIGEN3 ENCONTRADO")
message (WARNING "PATH EM ${Eigen3_DIR}")
endif()



add_library(KalmanLIB STATIC src/Kalman.cpp)
target_include_directories(KalmanLIB PUBLIC include)
target_link_libraries(KalmanLIB Eigen3::Eigen)

add_executable(Kalman src/main.cpp)
target_include_directories(Kalman PUBLIC include)
target_link_libraries(Kalman KalmanLIB Eigen3::Eigen)
