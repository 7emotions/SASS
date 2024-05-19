cmake_minimum_required(VERSION 3.8)
project(sr_core)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

file (GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.c
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

ament_auto_add_executable(${PROJECT_NAME} ${PROJECT_SOURCE})


install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()