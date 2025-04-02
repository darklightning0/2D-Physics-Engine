cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Specify macOS architecture
set(CMAKE_OSX_ARCHITECTURES "arm64") 

# Manually specify SFML directory if needed
set(SFML_DIR "/opt/homebrew/lib/cmake/SFML")

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Add the executable
add_executable(PhysicsEngine Codes/main.cpp Codes/World.cpp Codes/physics.cpp Codes/utility.cpp)

# Link SFML libraries
target_link_libraries(PhysicsEngine sfml-system sfml-window sfml-graphics)

# Include directories
include_directories(${SFML_INCLUDE_DIRS} /opt/homebrew/include)

set(CMAKE_CXX_FLAGS "-fsanitize=address -g")
