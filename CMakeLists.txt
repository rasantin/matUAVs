cmake_minimum_required(VERSION 3.20)

project(matUAVs VERSION 1.0 LANGUAGES CXX)

# ===============================
# C++ Standard
# ===============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ===============================
# Platform info
# ===============================
if(WIN32)
    message(STATUS "Building for Windows")
elseif(UNIX AND NOT APPLE)
    message(STATUS "Building for Linux")
elseif(APPLE)
    message(STATUS "Building for macOS")
endif()

# ===============================
# Compiler flags
# ===============================
if(MSVC)
    add_compile_options(/EHsc /W3)

    # 🔴 CRÍTICO: runtime compatível com Gurobi
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")
else()
    add_compile_options(-Wall -Wextra)
endif()

# ===============================
# Gurobi configuration
# ===============================
if(NOT DEFINED ENV{GUROBI_HOME})
    if(WIN32)
        set(ENV{GUROBI_HOME} "C:/gurobi1202/win64")
    else()
        set(ENV{GUROBI_HOME} "/opt/gurobi1202/linux64")
    endif()
endif()

set(GUROBI_DIR $ENV{GUROBI_HOME})
message(STATUS "GUROBI_HOME: ${GUROBI_DIR}")

# Headers
set(GUROBI_INCLUDE_DIR ${GUROBI_DIR}/include)

# Libraries
if(WIN32)
    find_library(GUROBI_CXX_LIBRARY
        NAMES gurobi_c++mt2017
        PATHS ${GUROBI_DIR}/lib
        REQUIRED
    )
    find_library(GUROBI_LIBRARY
        NAMES gurobi120
        PATHS ${GUROBI_DIR}/lib
        REQUIRED
    )
else()
    find_library(GUROBI_CXX_LIBRARY
        NAMES gurobi_c++
        PATHS ${GUROBI_DIR}/lib
        REQUIRED
    )
    find_library(GUROBI_LIBRARY
        NAMES gurobi120
        PATHS ${GUROBI_DIR}/lib
        REQUIRED
    )
endif()

# ===============================
# Sources
# ===============================
set(SOURCES
    src/Configuration.cpp
    src/Graph.cpp
    src/Input.cpp
    src/MHCP.cpp
    src/Node.cpp
    src/Output.cpp
    src/Rand.cpp
    src/Robot.cpp
    src/Solution.cpp
)

# ===============================
# Target
# ===============================
add_executable(matUAVs ${SOURCES})

target_include_directories(matUAVs PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${GUROBI_INCLUDE_DIR}
)

target_link_libraries(matUAVs PRIVATE
    ${GUROBI_CXX_LIBRARY}
    ${GUROBI_LIBRARY}
)

# Linux system libs
if(UNIX AND NOT APPLE)
    target_link_libraries(matUAVs PRIVATE pthread m)
endif()

# Output dir
set_target_properties(matUAVs PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "Configuration complete. Build with Release on Windows.")