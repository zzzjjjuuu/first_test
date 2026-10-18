cmake_minimum_required(VERSION 3.22)

# 设置 C 与 C++ 标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME C_project)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
add_compile_options(-gdwarf-4)
message("Build type: " ${CMAKE_BUILD_TYPE})

# 开启 C, C++ 与汇编支持
enable_language(C CXX ASM)

add_executable(${CMAKE_PROJECT_NAME})

# 引入 CubeMX 生成的底层库
add_subdirectory(cmake/stm32cubemx)

# 显式使用 ${CMAKE_CURRENT_SOURCE_DIR} 确保路径绝对正确
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/C_board_src/ControlTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/C_board_src/Gm6020.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/C_board_src/Pid.cpp
)

# 头文件包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/C_board_src
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 链接 CubeMX 库与数学库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
)
