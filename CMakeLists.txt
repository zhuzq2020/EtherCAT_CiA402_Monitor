cmake_minimum_required(VERSION 3.28)

project(EtherCATCiA402Monitor VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_GUI "Build the Qt 6 monitoring application" ON)
option(BUILD_TESTING "Build unit tests" ON)

add_library(cia402_state STATIC
    src/Cia402State.cpp
)
target_include_directories(cia402_state PUBLIC src)

if(BUILD_GUI)
    find_package(Qt6 6.2 REQUIRED COMPONENTS Core Widgets)

    set(SOEM_BUNDLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/soem")
    set(SOEM_LIBRARY "${SOEM_BUNDLE_DIR}/lib/libsoem.so")
    set(SOEM_INCLUDE_DIR "${SOEM_BUNDLE_DIR}/include")
    if(NOT EXISTS "${SOEM_LIBRARY}" OR NOT EXISTS "${SOEM_INCLUDE_DIR}/soem/soem.h")
        message(FATAL_ERROR "Bundled SOEM library or headers are missing from third_party/soem")
    endif()

    add_library(soem SHARED IMPORTED GLOBAL)
    set_target_properties(soem PROPERTIES
        IMPORTED_LOCATION "${SOEM_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${SOEM_INCLUDE_DIR}"
    )

    set(CMAKE_AUTOMOC ON)
    qt_add_executable(ethercat_cia402_monitor
        src/main.cpp
        src/MainWindow.h
        src/MainWindow.cpp
        src/EthercatWorker.h
        src/EthercatWorker.cpp
        src/Models.h
    )
    target_link_libraries(ethercat_cia402_monitor PRIVATE
        Qt6::Core
        Qt6::Widgets
        cia402_state
        soem
    )
    target_compile_options(ethercat_cia402_monitor PRIVATE -Wall -Wextra -Wpedantic)
    set_target_properties(ethercat_cia402_monitor PROPERTIES
        BUILD_RPATH "\$ORIGIN/../third_party/soem/lib"
        INSTALL_RPATH "\$ORIGIN/../lib"
    )

    install(TARGETS ethercat_cia402_monitor RUNTIME DESTINATION bin)
    install(FILES "${SOEM_LIBRARY}" DESTINATION lib)
endif()

if(BUILD_TESTING)
    enable_testing()
    add_executable(cia402_state_test tests/Cia402StateTest.cpp)
    target_link_libraries(cia402_state_test PRIVATE cia402_state)
    add_test(NAME cia402_state_test COMMAND cia402_state_test)
endif()
