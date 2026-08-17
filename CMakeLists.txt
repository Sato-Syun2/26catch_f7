cmake_minimum_required(VERSION 3.22)

project(26catch_f7 LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# STM32CubeMX generated application and HAL/FreeRTOS/LwIP dependencies.
add_executable(${PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

add_library(canlib_main STATIC
    canlib/CAN_Main/CAN_Main.c
    canlib/Defines/CAN_System_Def.c
)
target_include_directories(canlib_main PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/canlib/CAN_Main
    ${CMAKE_CURRENT_SOURCE_DIR}/canlib/Defines/Inc
)
target_link_libraries(canlib_main PUBLIC stm32cubemx)

# Build a CAN library from all C/C++/ASM sources below source_dir. Include
# directories are derived from the library's headers because its headers are
# split across several protocol-specific subdirectories.
function(add_canlib target_name source_dir)
    set(library_root "${CMAKE_CURRENT_SOURCE_DIR}/${source_dir}")

    if(NOT IS_DIRECTORY "${library_root}")
        message(FATAL_ERROR "CAN library directory not found: ${source_dir}")
    endif()

    file(GLOB_RECURSE library_sources CONFIGURE_DEPENDS
        "${library_root}/*.c"
        "${library_root}/*.cc"
        "${library_root}/*.cpp"
        "${library_root}/*.s"
        "${library_root}/*.S"
    )
    file(GLOB_RECURSE library_headers CONFIGURE_DEPENDS
        "${library_root}/*.h"
        "${library_root}/*.hh"
        "${library_root}/*.hpp"
    )

    if(NOT library_sources)
        message(FATAL_ERROR "No source files found for ${target_name}")
    endif()

    add_library(${target_name} STATIC ${library_sources})

    set(include_dirs "${library_root}")
    foreach(header_file IN LISTS library_headers)
        get_filename_component(header_dir "${header_file}" DIRECTORY)
        list(APPEND include_dirs "${header_dir}")
    endforeach()
    list(REMOVE_DUPLICATES include_dirs)

    target_include_directories(${target_name} PUBLIC ${include_dirs})

    # The CAN libraries use HAL types, CAN handles, and project headers.
    target_link_libraries(${target_name} PUBLIC stm32cubemx)
endfunction()

add_canlib(canlib_robomaster canlib_robomaster)
add_canlib(canlib_robstride canlib_robstride)
# Robstride の制限値設定で fminf/fmaxf を使用する。
# 静的ライブラリの後にリンクされるよう依存先として宣言する。
target_link_libraries(canlib_robstride PUBLIC m)

# micro-ROS の事前生成済み静的ライブラリを CMake ターゲットとして登録する。
# このライブラリは STM32/FreeRTOS 向けに生成されているため、ホスト用の
# find_package() は使わず、同梱されている libmicroros.a を直接リンクする。
set(MICROROS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_stm32cubemx_utils")
set(MICROROS_LIBRARY "${MICROROS_ROOT}/microros_static_library/libmicroros/libmicroros.a")
set(MICROROS_INCLUDE_DIR "${MICROROS_ROOT}/microros_static_library/libmicroros/microros_include")

if(NOT EXISTS "${MICROROS_LIBRARY}")
    message(FATAL_ERROR "micro-ROS static library not found: ${MICROROS_LIBRARY}")
endif()
if(NOT IS_DIRECTORY "${MICROROS_INCLUDE_DIR}")
    message(FATAL_ERROR "micro-ROS include directory not found: ${MICROROS_INCLUDE_DIR}")
endif()

add_library(microros STATIC IMPORTED GLOBAL)
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION "${MICROROS_LIBRARY}"
)
target_include_directories(microros INTERFACE "${MICROROS_INCLUDE_DIR}")

target_sources(${PROJECT_NAME} PRIVATE
    Core/Src/can_devices.c
    # micro-ROS を STM32CubeMX/FreeRTOS/LwIP 上で動かすための補助実装。
    ${MICROROS_ROOT}/extra_sources/custom_memory_manager.c
    ${MICROROS_ROOT}/extra_sources/microros_allocators.c
    ${MICROROS_ROOT}/extra_sources/microros_time.c
    ${MICROROS_ROOT}/extra_sources/microros_transports/udp_transport.c
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
    # udp_transport.c を有効化し、micro-ROS にカスタム UDP 通信を使わせる。
    RMW_UXRCE_TRANSPORT_CUSTOM
)

# STM32F767 is a Cortex-M7 with an FPv5 single-precision FPU.
target_compile_options(${PROJECT_NAME} PRIVATE
    -mfloat-abi=hard
    -mfpu=fpv5-d16
)

target_link_options(${PROJECT_NAME} PRIVATE
    -mfloat-abi=hard
    -mfpu=fpv5-d16
    -u _printf_float
)

# cmake/stm32cubemx/CMakeLists.txt uses the plain signature for this target,
# so keep the same signature here for compatibility.
target_link_libraries(${PROJECT_NAME}
    stm32cubemx
    canlib_main
    canlib_robomaster
    canlib_robstride
    microros
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
