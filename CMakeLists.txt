cmake_minimum_required(VERSION 3.12)
project(matUAVs VERSION 1.0 LANGUAGES CXX)

# Configurações do C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Mensagens informativas sobre a plataforma
if(WIN32)
    message(STATUS "Building for Windows")
elseif(UNIX AND NOT APPLE)
    message(STATUS "Building for Linux")
elseif(APPLE)
    message(STATUS "Building for macOS")
endif()

# Configurações específicas do compilador
if(MSVC)
    # Visual Studio: Enable exception handling
    add_compile_options(/EHsc)
    # Ativar avisos úteis
    add_compile_options(/W3)
else()
    # GCC/Clang: Enable warnings
    add_compile_options(-Wall -Wextra)
endif()

# Busca pelo Gurobi (obrigatório)
# Define GUROBI_HOME se não estiver definido
if(NOT DEFINED ENV{GUROBI_HOME})
    if(WIN32)
        # Caminho padrão no Windows
        set(ENV{GUROBI_HOME} "C:/gurobi1202/win64")
    elseif(UNIX)
        # Caminho padrão no Linux
        set(ENV{GUROBI_HOME} "/opt/gurobi1202/linux64")
    endif()
endif()

# Variáveis para Gurobi
set(GUROBI_DIR $ENV{GUROBI_HOME})
message(STATUS "GUROBI_DIR: ${GUROBI_DIR}")

# Incluir headers do Gurobi
include_directories(${GUROBI_DIR}/include)

# Bibliotecas do Gurobi
if(WIN32)
    # Windows: usa gurobi_c++mt2017.lib e gurobi120.lib
    find_library(GUROBI_CXX_LIBRARY
        NAMES gurobi_c++mt2017
        HINTS ${GUROBI_DIR}/lib
    )
    find_library(GUROBI_LIBRARY
        NAMES gurobi120 gurobi110 gurobi100
        HINTS ${GUROBI_DIR}/lib
    )
else()
    # Linux: usa libgurobi_c++.a e libgurobi120.so
    find_library(GUROBI_CXX_LIBRARY
        NAMES gurobi_c++
        HINTS ${GUROBI_DIR}/lib
    )
    find_library(GUROBI_LIBRARY
        NAMES gurobi120 gurobi110 gurobi100
        HINTS ${GUROBI_DIR}/lib
    )
endif()

if(NOT GUROBI_CXX_LIBRARY OR NOT GUROBI_LIBRARY)
    message(FATAL_ERROR "Gurobi libraries not found. Please set GUROBI_HOME environment variable.")
endif()

message(STATUS "Gurobi C++ library: ${GUROBI_CXX_LIBRARY}")
message(STATUS "Gurobi library: ${GUROBI_LIBRARY}")

# Lista de arquivos fonte
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

# Cria o executável
add_executable(main ${SOURCES})

# Link com as bibliotecas do Gurobi
target_link_libraries(main
    ${GUROBI_CXX_LIBRARY}
    ${GUROBI_LIBRARY}
)

# Bibliotecas adicionais específicas do sistema
if(UNIX AND NOT APPLE)
    # Linux: adiciona pthread e m (math)
    target_link_libraries(main pthread m)
elseif(WIN32)
    # Windows: pode precisar de bibliotecas adicionais se necessário
endif()

# Define o diretório de saída dos binários
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Cria diretório para logs
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/logs)

# Mensagem final
message(STATUS "Configuration complete. Build with: cmake --build .")
