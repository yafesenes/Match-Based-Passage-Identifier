cmake_minimum_required(VERSION 3.16)
project(Match-Based-Passage-Identifier)

set(CMAKE_CXX_STANDARD 17)

# Add source files
file(GLOB_RECURSE SOURCE_FILES
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
)
add_library(MBPI SHARED ${SOURCE_FILES})

find_package(Boost REQUIRED COMPONENTS system)
target_include_directories(MBPI PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(MBPI PUBLIC ${Boost_LIBRARIES})

find_package(OpenCV REQUIRED)
target_include_directories(MBPI PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(MBPI PUBLIC ${OpenCV_LIBS})

find_package(ompl REQUIRED)
target_include_directories(MBPI PUBLIC ${OMPL_INCLUDE_DIRS})
target_link_libraries(MBPI PUBLIC ${OMPL_LIBRARIES})

file(GLOB_RECURSE SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp")
target_sources(MBPI PRIVATE ${SOURCE_FILES})

target_include_directories(MBPI PUBLIC ${PROJECT_SOURCE_DIR}/include)
