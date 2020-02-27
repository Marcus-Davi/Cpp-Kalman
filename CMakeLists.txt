cmake_minimum_required(VERSION 3.8)
project (KalmanEigen)

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) #útil pra YCM (youcompleteme)

file(GLOB SOURCES "src/*.cpp")
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

if(EIGEN3_FOUND)
message (WARNING "EIGEN3 ENCONTRADO")
message (WARNING "PATH EM ${Eigen3_DIR}")
endif()


add_executable(Kalman ${SOURCES})
#add_executable(Kalman src/main.cpp src/Kalman.cpp)

target_link_libraries (Kalman Eigen3::Eigen)
target_include_directories(Kalman PUBLIC include)




