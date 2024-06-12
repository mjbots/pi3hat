# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com

# moteus does not use cmake to build, but does expose some client
# libraries for use in cmake projects.

# A client project can use this as follows:
#
# include(FetchContent)
# FetchContent_Declare(
#   pi3hat
#   GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
#   GIT_TAG        00112233445566778899aabbccddeeff00112233
# )
#
# set(CMAKE_CXX_FLAGS "-march=native -mcpu=native -mtune=native")
# FetchContent_MakeAvailable(pi3hat)
#
# add_executable(myproject myproject.cc)
# target_link_libraries(myproject pi3hat::pi3hat)


cmake_minimum_required(VERSION 3.10)
project(moteus VERSION 0.1.20230913)

set(MAIN_PROJECT OFF)

add_library(pi3hat_internal
  lib/cpp/mjbots/pi3hat/pi3hat.cc
  lib/cpp/examples/pi3hat_moteus_transport_register.cc
)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  # This git hash can (and likely should) be overriden by the higher
  # level project.
  GIT_TAG        25c847a7b7b739aa00d4f9a1a30fb041f2ce7a7b
)

FetchContent_MakeAvailable(moteus)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(pi3hat_internal PRIVATE Threads::Threads)

add_library(pi3hat INTERFACE)
add_library(pi3hat::pi3hat ALIAS pi3hat)

target_include_directories(pi3hat_internal PUBLIC lib/cpp/mjbots/pi3hat)
target_link_libraries(pi3hat_internal PUBLIC moteus::cpp)
target_link_libraries(pi3hat_internal PUBLIC -lbcm_host)

if (${CMAKE_VERSION} VERSION_GREATER "3.24.0")
  # We wrap our "internal" library in an external one so that we can
  # apply -Wl,--whole-archive to the external one.  Otherwise, the
  # pi3hat_moteus_transport_register.cc symbols will get dropped on the
  # floor.
  target_link_libraries(
    pi3hat INTERFACE "$<LINK_LIBRARY:WHOLE_ARCHIVE,pi3hat_internal>")
else()
  # If our cmake is too old, we'll just let the symbols get dropped
  # and the user will have to call
  # 'mjbots::pi3hat::Pi3HatMoteusFactory::Register()' by hand.
  target_link_libraries(pi3hat INTERFACE pi3hat_internal)
endif()
