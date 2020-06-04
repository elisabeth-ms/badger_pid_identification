project(badger_pid_identification)

cmake_minimum_required(VERSION 2.8)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/fcontrol/)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/plot/)
INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})

add_executable(online_joint_identification_data src/online_joint_identification_data.cpp)

target_link_libraries(online_joint_identification_data ${SUBDIR_LINK_NAMES})