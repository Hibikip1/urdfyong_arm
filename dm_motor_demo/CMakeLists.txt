cmake_minimum_required(VERSION 3.8)
project(dm_motor_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# DM SDK paths (relative to this CMakeLists.txt)
set(DM_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../dm-device-sdk/sdk-lib")
set(DM_SDK_INCLUDE "${DM_SDK_DIR}")
set(DM_SDK_LIB "${DM_SDK_DIR}/linux/x86_64/libdm_device.so")

add_executable(${PROJECT_NAME} dm_motor_demo.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${DM_SDK_INCLUDE})

target_link_libraries(${PROJECT_NAME}
  ${DM_SDK_LIB}
  pthread
)

# Set RPATH so the binary finds libdm_device.so at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${DM_SDK_DIR}/linux/x86_64"
  INSTALL_RPATH "${DM_SDK_DIR}/linux/x86_64"
)
