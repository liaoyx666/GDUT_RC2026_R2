cmake_minimum_required(VERSION 4.2.1)
project(RC_2026)
set(CMAKE_CXX_STANDARD 17)


set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
add_compile_options(-Wall -Wextra)

include_directories(${PROJECT_SOURCE_DIR}/include)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


set(PROJECT_SRC 
    "${PROJECT_SOURCE_DIR}/src/Nine_Square/Game_Strategy.cpp"
)

add_executable(RC_2026 main.cpp ${PROJECT_SRC})
