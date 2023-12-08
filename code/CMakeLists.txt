cmake_minimum_required(VERSION 3.22.1)

project("tspd" VERSION 1.0 DESCRIPTION "Decremental State-Space Relaxations for the Basic Traveling Salesman Problem with a Drone")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -m64")
if ("${CMAKE_BUILD_TYPE}" MATCHES release OR "${CMAKE_BUILD_TYPE}" MATCHES Release)
    message("-- Compiling in Release Mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
elseif ("${CMAKE_BUILD_TYPE}" MATCHES debug OR "${CMAKE_BUILD_TYPE}" MATCHES Debug)
    message("-- Compiling in Debug Mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# If the compilation is called from RUNNER then set the macro constant RUNNER.
if ("${RUNNER}" MATCHES ON)
    message("-- Runner compilation")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUNNER")
endif()

# Add include directory.
include_directories("include/")

# Including GOC.
add_subdirectory(goc)
include_directories(${PROJECT_NAME} SYSTEM "goc/include/")

# Create binaries.
add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE  
    src/bp_solver.cpp 
    src/bit_graph.cpp 
    src/labeling.cpp 
    src/mcmf_heuristic.cpp
    src/ng_utils.cpp
    src/route.cpp 
    src/scf.cpp 
    src/solver.cpp 
    src/subinstance.cpp 
    src/time_guardian.cpp
    src/tspd_instance.cpp 
    src/log/labeling_log.cpp 
    src/log/bp_solver_log.cpp
    src/log/solver_log.cpp
    src/log/solver_problem_log.cpp
    src/log/variable_fixing_log.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE goc)
#target_link_libraries(tspd)
#target_compile_options(tspd PRIVATE -Werror -Wall -Wextra -Wpedantic -Wnull-dereference -Wcast-qual -Wconversion)
target_compile_options(${PROJECT_NAME} PRIVATE -Werror -Wall -Wextra -Wpedantic -Wnull-dereference -Wcast-qual -Wlogical-op -Wduplicated-cond -Wshadow -Wconversion)

add_executable(tspd_pfc src/main_pfc.cpp)
target_link_libraries(tspd_pfc PUBLIC ${PROJECT_NAME})

add_executable(tspd_bp src/main_bp.cpp)
target_link_libraries(tspd_bp PUBLIC ${PROJECT_NAME})

add_executable(tspd_lbl src/main_lbl.cpp)
target_link_libraries(tspd_lbl PUBLIC ${PROJECT_NAME})

add_executable(tspd_vf src/main_vf.cpp)
target_link_libraries(tspd_vf PUBLIC ${PROJECT_NAME})

add_executable(poi_to_json src/poi_to_json.cpp)
target_link_libraries(poi_to_json PUBLIC ${PROJECT_NAME})
