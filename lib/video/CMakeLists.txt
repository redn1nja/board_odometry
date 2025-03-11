cmake_minimum_required(VERSION 3.5)

project(video CXX)

set (CMAKE_CXX_STANDARD 14)


find_package(PkgConfig QUIET)
pkg_check_modules(RKAIQ QUIET "rkaiq")

include(FindPackageHandleStandardArgs)
find_path(RKAIQ_INCLUDE_DIR
        NAMES
        common/rk_aiq.h
        PATH
        include
        PATH_SUFFIXES
        rkaiq)
find_library(RKAIQ_LIBRARY
        rkaiq)
find_package_handle_standard_args(RKAIQ
        DEFAULT_MSG
        RKAIQ_INCLUDE_DIR
        RKAIQ_LIBRARY
        )
mark_as_advanced(
        RKAIQ_INCLUDE_DIR
        RKAIQ_LIBRARY)

if(RKAIQ_FOUND)
    set(RKAIQ_LIBRARIES    ${RKAIQ_LIBRARY})
    set(RKAIQ_INCLUDE_DIRS
            ${RKAIQ_INCLUDE_DIR}/algos;
            ${RKAIQ_INCLUDE_DIR}/algos/a3dlut;
            ${RKAIQ_INCLUDE_DIR}/algos/ablc;
            ${RKAIQ_INCLUDE_DIR}/algos/accm;
            ${RKAIQ_INCLUDE_DIR}/algos/acp;
            ${RKAIQ_INCLUDE_DIR}/algos/adebayer;
            ${RKAIQ_INCLUDE_DIR}/algos/adehaze;
            ${RKAIQ_INCLUDE_DIR}/algos/adpcc;
            ${RKAIQ_INCLUDE_DIR}/algos/ae;
            ${RKAIQ_INCLUDE_DIR}/algos/af;
            ${RKAIQ_INCLUDE_DIR}/algos/agamma;
            ${RKAIQ_INCLUDE_DIR}/algos/ahdr;
            ${RKAIQ_INCLUDE_DIR}/algos/aie;
            ${RKAIQ_INCLUDE_DIR}/algos/alsc;
            ${RKAIQ_INCLUDE_DIR}/algos/anr;
            ${RKAIQ_INCLUDE_DIR}/algos/aorb;
            ${RKAIQ_INCLUDE_DIR}/algos/asd;
            ${RKAIQ_INCLUDE_DIR}/algos/asharp;
            ${RKAIQ_INCLUDE_DIR}/algos/awb;
            ${RKAIQ_INCLUDE_DIR}/common;
            ${RKAIQ_INCLUDE_DIR}/iq_parser;
            ${RKAIQ_INCLUDE_DIR}/uAPI;
            ${RKAIQ_INCLUDE_DIR}/xcore)

    if(RKAIQ_LIBRARY AND NOT TARGET RkAiq::RkAiq)
        if(IS_ABSOLUTE "${RKAIQ_LIBRARY}")
            add_library(RkAiq::RkAiq UNKNOWN IMPORTED)
            set_target_properties(RkAiq::RkAiq PROPERTIES IMPORTED_LOCATION
                    "${RKAIQ_LIBRARY}")
        else()
            add_library(RkAiq::RkAiq INTERFACE IMPORTED)
            set_target_properties(RkAiq::RkAiq PROPERTIES IMPORTED_LIBNAME
                    "${RKAIQ_LIBRARY}")
        endif()
        set_target_properties(RkAiq::RkAiq PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                "${RKAIQ_INCLUDE_DIR}")
    endif()
endif()


find_package(OpenCV REQUIRED core imgproc)
if (OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    message(STATUS "OpenCV path: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
    include_directories(${OpenCV_INCLUDE_DIRS})

    # Additional Library Directories
    link_directories( ${OpenCV_LIB_DIR} )

else()
    message("OpenCV not found.")
endif()


set (SRC 
        ${CMAKE_CURRENT_SOURCE_DIR}/Camera.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/ISP.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/Stream.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/VideoOutput.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/VideoChannel.h
        ${CMAKE_CURRENT_SOURCE_DIR}/RGA.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/Display.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/Common.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/Overlay.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/DrawingOverlay.cc
)

if (COMPILE_SHARED_LIBRARIES)
    add_library(${PROJECT_NAME} SHARED ${SRC})
else()
    add_library(${PROJECT_NAME} STATIC ${SRC})
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/lib/tracker)
target_include_directories(${PROJECT_NAME} PUBLIC ${RKAIQ_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE rkaiq easymedia pthread ${OpenCV_LIBS})
