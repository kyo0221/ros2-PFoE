cmake_minimum_required(VERSION 3.8)
project(pfoe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build C++ library
ament_auto_add_library(pfoe_lib
  src/Observation.cpp
  src/Event.cpp
  src/Episodes.cpp
  src/Particle.cpp
  src/ParticleFilter.cpp
)

# Build replay executable
ament_auto_add_executable(replay src/replay.cpp)
target_link_libraries(replay pfoe_lib)

# Install Python scripts
install(PROGRAMS
  scripts/joy_controller.py
  scripts/feature_extractor.py
  scripts/logger.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config weights)
