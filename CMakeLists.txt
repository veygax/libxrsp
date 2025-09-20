cmake_minimum_required(VERSION 3.15)

project(
  libxrsp
  VERSION 1.0
  LANGUAGES CXX)

file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.42.0/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=2020b4fc42dba44817983e06342e682ecfc3d2f484a581f11cc5731fbe4dce8a
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

CPMAddPackage("gh:libusb/libusb-cmake#v1.0.29-0")
CPMAddPackage("gh:fmtlib/fmt#12.0.0")

add_library(xrsp SHARED src/xrsp.cc src/xrsp.h)

target_link_libraries(xrsp PRIVATE usb-1.0 fmt::fmt)