cmake_minimum_required(VERSION 3.23)
set(PROJECT_NAME PhysicsEngine)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)

set(source_dir ${PROJECT_SOURCE_DIR}/src)
set(libs_dir ${PROJECT_SOURCE_DIR}/libs)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find all .h and .cpp files:
file(GLOB_RECURSE source_files ${source_dir}/*.cpp)
file(GLOB_RECURSE include_files ${source_dir}/*.h)
# find all subdirectories to include
file(GLOB_RECURSE all_files_folders LIST_DIRECTORIES true ${source_dir}/*)
SET(include_dirs "")
FOREACH(file_or_folder ${all_files_folders})
    IF(IS_DIRECTORY ${file_or_folder})
        LIST(APPEND include_dirs ${file_or_folder})
    ENDIF()
ENDFOREACH()

include_directories(${PROJECT_NAME}
        ${source_dir}
        ${include_dirs}
        ${libs_dir})

add_executable(${PROJECT_NAME} ${source_files} ${include_files} main.cpp)
