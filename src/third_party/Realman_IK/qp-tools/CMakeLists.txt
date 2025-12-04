cmake_minimum_required(VERSION 3.5)

project(
    qpSWIFT
    VERSION 1.00
    DESCRIPTION "qpSWIFT: A Sparse Quadratic Program Solver"
    )


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/exec)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/arc)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/lib)


#### ---- Options ---- ####
option (QPTESTS "Build Unit Tests" OFF)
option (QPSHAREDLIB "Build Unit Tests" ON)
option (QPDEMOS "Build C and C++ Demos" ON)


#### ----- CXX Standard Flags ----- #####
### Include common install dirs
include(GNUInstallDirs)

set(EXTRA_LIBS "")
if(NOT MSVC)
### Add Standard Math Library 
        LIST(APPEND EXTRA_LIBS "-lm")
## For Linux add realtime library
        if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
                LIST(APPEND EXTRA_LIBS "-lrt -ldl")
        endif ()
endif(NOT MSVC)


### Turn on fPIC option on
set(CMAKE_POSITION_INDEPENDENT_CODE ON) 

### Default Build Type Release
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

#### ----- CXX Standard Flags ----- #####


add_subdirectory(src)
add_subdirectory(include)

if(QPTESTS)
        enable_testing()      
        add_subdirectory(tests)
endif()


### --- Create Static Library
add_library(qpSWIFT-static STATIC ${qpSWIFT_src})
target_link_libraries(qpSWIFT-static ${EXTRA_LIBS})
set_target_properties(qpSWIFT-static PROPERTIES OUTPUT_NAME qpSWIFT)
target_compile_options(qpSWIFT-static PRIVATE -O3)
target_include_directories(qpSWIFT-static PUBLIC
                "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/qpSWIFT>"
)

### --- Create Shared Library
if(QPSHAREDLIB)
        add_library(qpSWIFT-shared SHARED ${qpSWIFT_src})
        target_link_libraries(qpSWIFT-shared ${EXTRA_LIBS})
        set_target_properties(qpSWIFT-shared PROPERTIES OUTPUT_NAME qpSWIFT)
        target_include_directories(qpSWIFT-shared PUBLIC 
                        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                        "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/qpSWIFT>"
)
endif()


if(QPDEMOS)
### --- Build C Exectuable
        add_executable(runqp)
        target_sources(runqp PRIVATE ${PROJECT_SOURCE_DIR}/demo/runqp.c)
        target_link_libraries(runqp PRIVATE qpSWIFT-static)

### --- Build Cpp Executable
        add_executable(runqpcpp)
        target_sources(runqpcpp PRIVATE ${PROJECT_SOURCE_DIR}/demo/runqpcpp.cpp)
        target_link_libraries(runqpcpp PRIVATE qpSWIFT-static) 
endif()


### ---- Install Static Library ------- ####
install(TARGETS qpSWIFT-static
        EXPORT "${PROJECT_NAME}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

### ---- Install Shared Library ------- ####
if(QPSHAREDLIB)
        install(TARGETS qpSWIFT-shared
                EXPORT "${PROJECT_NAME}"
                ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
                LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
                RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
                )
endif()


install(FILES ${qpSWIFT_headers} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/qpSWIFT")


### ---- Create CMake Package for exporting the target ####
include(CMakePackageConfigHelpers)

export(EXPORT ${PROJECT_NAME}
  FILE "${CMAKE_CURRENT_BINARY_DIR}/qpSWIFT-targets.cmake"
  NAMESPACE qpSWIFT::
)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/qpSWIFTConfig.cmake "include(\"\${CMAKE_CURRENT_LIST_DIR}/qpSWIFT-targets.cmake\")\n")

# Create CMake packages for the install directory
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/cmake/qpSWIFT)

install(EXPORT ${PROJECT_NAME}
        FILE qpSWIFT-targets.cmake
        NAMESPACE qpSWIFT::
        DESTINATION ${ConfigPackageLocation}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/qpSWIFTConfig.cmake
        DESTINATION ${ConfigPackageLocation}
)
