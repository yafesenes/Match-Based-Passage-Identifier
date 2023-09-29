cmake_minimum_required(VERSION 3.16)
project(NarrowPassageRemake)

set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/lib/SFML-2.5.1/lib/cmake/SFML")
find_package(SFML 2.5.1 COMPONENTS system window graphics network audio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Add source files
file(GLOB_RECURSE SOURCE_FILES
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries (${PROJECT_NAME} Threads::Threads yaml-cpp sfml-graphics sfml-window sfml-system ${Boost_LIBRARIES} ${OpenCV_LIBS})
