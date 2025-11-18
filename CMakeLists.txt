cmake_minimum_required(VERSION 3.25)
project(Physik VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# fetch Eigen
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Disable Eigen tests" FORCE)
set(EIGEN_MPL2_ONLY ON CACHE BOOL "Use MPL2-only parts" FORCE)

FetchContent_MakeAvailable(eigen)

# add files recursively
file(GLOB_RECURSE ENGINE_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
)

# create static lib
add_library(${PROJECT_NAME} STATIC ${ENGINE_SOURCES})

# add compile flags
#if (MSVC)
#    target_compile_options(${PROJECT_NAME} PRIVATE
#        /W4
#        /WX
#    )
#else()
#    target_compile_options(${PROJECT_NAME} PRIVATE
#        -Wall
#        -Wextra
#        -Wpedantic
#        -Wconversion
#        -Werror
#    )
#endif()

# expose headers in src to consumers
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(Physik PUBLIC Eigen3::Eigen)
