cmake_minimum_required(VERSION 3.20)
project(learning_enhanced_tracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(let_track
  src/main.cpp
  src/sim/sim.cpp
  src/tracking/kalman.cpp
)

target_include_directories(let_track PRIVATE src)
target_compile_options(let_track PRIVATE -Wall -Wextra -Wpedantic)