cmake_minimum_required(VERSION 3.16)
project(arm_maint_tool C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# SOEM 源码目录
set(SOEM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/soem")

set(SOEM_SOURCES
    "${SOEM_DIR}/src/ec_base.c"
    "${SOEM_DIR}/src/ec_coe.c"
    "${SOEM_DIR}/src/ec_config.c"
    "${SOEM_DIR}/src/ec_dc.c"
    "${SOEM_DIR}/src/ec_eoe.c"
    "${SOEM_DIR}/src/ec_foe.c"
    "${SOEM_DIR}/src/ec_main.c"
    "${SOEM_DIR}/src/ec_print.c"
    "${SOEM_DIR}/src/ec_soe.c"
    "${SOEM_DIR}/osal/linux/osal.c"
    "${SOEM_DIR}/oshw/linux/nicdrv.c"
    "${SOEM_DIR}/oshw/linux/oshw.c"
)

add_library(soem_static STATIC ${SOEM_SOURCES})
target_include_directories(soem_static PUBLIC
    "${SOEM_DIR}/include"
    "${SOEM_DIR}/osal"
    "${SOEM_DIR}/osal/linux"
    "${SOEM_DIR}/oshw/linux"
)

# 通用机械臂 EtherCAT (SOEM) 控制 SDK 核心库
add_library(arm_soem_sdk STATIC
    src/soem_bus.cpp
    src/drive_profile.cpp
    src/arm_controller.cpp
    src/pdo_exchange.cpp
    src/eeprom_access.cpp
    src/rpc_session.cpp
)
target_include_directories(arm_soem_sdk PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(arm_soem_sdk PUBLIC
    soem_static
    pthread
)
target_compile_options(arm_soem_sdk PRIVATE -Wall -Wextra -Werror)

# 底层维护与实时控制可执行文件
add_executable(arm_maint
    src/main.cpp
)
target_link_libraries(arm_maint PRIVATE
    arm_soem_sdk
)
target_compile_options(arm_maint PRIVATE -Wall -Wextra -Werror)

install(TARGETS arm_maint DESTINATION bin)

include(CTest)
if(BUILD_TESTING)
    add_executable(pdo_exchange_test tests/pdo_exchange_test.cpp src/pdo_exchange.cpp)
    target_include_directories(pdo_exchange_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_link_libraries(pdo_exchange_test PRIVATE soem_static pthread)
    target_link_options(pdo_exchange_test PRIVATE
        -Wl,--wrap=ecx_getindex -Wl,--wrap=ecx_setbufstat
        -Wl,--wrap=ecx_outframe_red -Wl,--wrap=ecx_waitinframe)
    add_test(NAME pdo_exchange COMMAND pdo_exchange_test)

    add_executable(motion_pdo_group_test tests/motion_pdo_group_test.cpp)
    target_include_directories(motion_pdo_group_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_link_libraries(motion_pdo_group_test PRIVATE soem_static)
    add_test(NAME motion_pdo_group COMMAND motion_pdo_group_test)

    add_executable(csp_mode_test tests/csp_mode_test.cpp)
    target_include_directories(csp_mode_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    add_test(NAME csp_mode COMMAND csp_mode_test)

    add_executable(sdk_core_test tests/sdk_core_test.cpp)
    target_link_libraries(sdk_core_test PRIVATE arm_soem_sdk)
    add_test(NAME sdk_core COMMAND sdk_core_test)

    add_executable(sii_integrity_test tests/sii_integrity_test.cpp)
    target_include_directories(sii_integrity_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    add_test(NAME sii_integrity COMMAND sii_integrity_test)

    add_executable(eeprom_access_test tests/eeprom_access_test.cpp src/eeprom_access.cpp)
    target_include_directories(eeprom_access_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_compile_options(eeprom_access_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME eeprom_access COMMAND eeprom_access_test)

    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME python_workflows_cli COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/alias_cli_test.py" "$<TARGET_FILE:arm_maint>")
    endif()
endif()
