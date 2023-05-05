cmake_minimum_required(VERSION 3.21)
project(libxdc LANGUAGES C)

if (CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(LIBXDC_ROOT_PROJECT ON)
else()
    set(LIBXDC_ROOT_PROJECT OFF)
endif()

option(LIBXDC_BUILD_SHARED "Build shared libxdc library" OFF)
option(LIBXDC_INSTALL "Generate install rules" ${LIBXDC_ROOT_PROJECT})

set(LIBXDC_CAPSTONE_INSTALL_DIR "" CACHE PATH "Path to capstone's install directory.")
mark_as_advanced(LIBXDC_CAPSTONE_INSTALL_DIR)

if (LIBXDC_ROOT_PROJECT OR NOT capstone_FOUND)
    if (LIBXDC_CAPSTONE_INSTALL_DIR)
        list(APPEND CMAKE_PREFIX_PATH "${LIBXDC_CAPSTONE_INSTALL_DIR}")
    endif()
    find_package(capstone REQUIRED)
endif()

set(libxdc_headers libxdc.h)
set(libxdc_sources
    src/cfg.c
    src/cfg.h
    src/core.h
    src/decoder.c
    src/decoder.h
    src/disassembler.c
    src/disassembler.h
    src/khash.h
    src/libxdc.c
    src/libxdc.h
    src/mmh3.c
    src/mmh3.h
    src/tnt_cache.c
    src/tnt_cache.h
    src/trace_cache.c
    src/trace_cache.h
)

set(libxdc_installed_headers include/libxdc.h)

function (add_libxdc_library target_name library_type)
    add_library("${target_name}" "${library_type}")
    target_sources("${target_name}"
        PRIVATE ${libxdc_sources} ${libxdc_header}
        INTERFACE
            $<BUILD_INTERFACE:${libxdc_header}>
            $<INCLUDE_INTERFACE:${libxdc_installed_headers}>
    )
    set_target_properties("${target_name}" PROPERTIES OUTPUT_NAME xdc)

    target_link_libraries("${target_name}" PRIVATE capstone::capstone)

    # When performing a release build with GCC-compatible compilers (namely, GCC and clang), add some
    # optimization-related options to the compile command.
    target_compile_options("${target_name}" PRIVATE
        $<$<AND:$<CONFIG:Release>,$<C_COMPILER_ID:Clang,GNU>>:-Ofast;-fvisibility=hidden;-finline-functions>
    )

    # Enable LTO in release builds.
    set_target_properties("${target_name}" PROPERTIES
        INTERPROCEDURAL_OPTIMIZATION $<CONFIG:RELEASE>
    )

    if (LIBXDC_INSTALL)
        # Install library files
        install(TARGETS "${target_name}"
            EXPORT libxdc-targets
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            # Windows DLLs are classified as RUNTIME.
            RUNTIME DESTINATION lib
        )
    endif()
endfunction()

if (LIBXDC_BUILD_SHARED)
    add_libxdc_library(libxdc_shared SHARED)
    target_compile_definitions(libxdc_shared PUBLIC LIBXDC_BUILD_SHARED)
else()
    add_libxdc_library(libxdc_static STATIC)
endif()

if (LIBXDC_INSTALL)
    # Generate libxdc-config.cmake file for others to use find_package
    set(libxdc_cmake_config_install_dir "${CMAKE_INSTALL_PREFIX}/cmake/libxdc")
    include(CMakePackageConfigHelpers)
    configure_package_config_file(
        libxdc-config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/libxdc-config.cmake"
        INSTALL_DESTINATION "${libxdc_cmake_config_install_dir}"
    )

    # Install the main header file.
    install(FILES ${libxdc_headers} DESTINATION include)

    # Export targets to libxdc-targets.cmake
    install(EXPORT libxdc-targets
        NAMESPACE libxdc::
        DESTINATION "${libxdc_cmake_config_install_dir}"
    )

    # Install libxdc-config.cmake
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libxdc-config.cmake"
        DESTINATION "${libxdc_cmake_config_install_dir}"
    )
endif()
