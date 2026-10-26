cmake_minimum_required(VERSION 3.16)
project(arm_maint_tool C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)

# 编译优化
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

# 静态库目标
add_library(soem_static STATIC ${SOEM_SOURCES})

target_include_directories(soem_static PUBLIC
    "${SOEM_DIR}/include"
    "${SOEM_DIR}/osal"
    "${SOEM_DIR}/osal/linux"
    "${SOEM_DIR}/oshw/linux"
)

# 编译单一二进制维护工具
add_executable(arm_maint
    src/main.cpp
    src/maint_engine.cpp
    src/eeprom_access.cpp
)

target_include_directories(arm_maint PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${SOEM_DIR}/include"
    "${SOEM_DIR}/osal"
    "${SOEM_DIR}/osal/linux"
    "${SOEM_DIR}/oshw/linux"
)

target_link_libraries(arm_maint PRIVATE
    soem_static
    pthread
)

# 安装规则
install(TARGETS arm_maint DESTINATION bin)

include(CTest)
if(BUILD_TESTING)
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME alias_cli COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/alias_cli_test.py" "$<TARGET_FILE:arm_maint>")
    endif()
    add_executable(sii_integrity_test tests/sii_integrity_test.cpp)
    target_include_directories(sii_integrity_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    add_test(NAME sii_integrity COMMAND sii_integrity_test)
    add_executable(eeprom_access_test tests/eeprom_access_test.cpp src/eeprom_access.cpp)
    target_include_directories(eeprom_access_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_compile_options(eeprom_access_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME eeprom_access COMMAND eeprom_access_test)
endif()
