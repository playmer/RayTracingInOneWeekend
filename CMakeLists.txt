project(RayTracingInOneWeekend CXX)

cmake_minimum_required(VERSION 3.9.1 FATAL_ERROR) 

add_executable(RayTracing Source.cpp)

target_include_directories(RayTracing PUBLIC Dependencies/glm)
target_include_directories(RayTracing PUBLIC Dependencies)

if (MSVC)
  target_compile_definitions(RayTracing PUBLIC -D_CRT_SECURE_NO_WARNINGS)
endif()


set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT RayTracing) 