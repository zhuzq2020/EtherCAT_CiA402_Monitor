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
