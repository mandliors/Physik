cmake_minimum_required(VERSION 3.25)
project(Physik VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add files recursively
file(GLOB_RECURSE ENGINE_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
)

# create static lib
add_library(${PROJECT_NAME} STATIC ${ENGINE_SOURCES})

# add compile flags
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W4
        /WX
    )
else()
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Werror
    )
endif()

# create source groups for vs
foreach(FILE ${ENGINE_SOURCES})
    file(RELATIVE_PATH REL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src ${FILE})
    get_filename_component(DIR ${REL_FILE} PATH)
    string(REPLACE "/" "\\" DIR_WIN ${DIR})
    source_group("${DIR_WIN}" FILES ${FILE})
endforeach()

# expose headers in src to consumers
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
