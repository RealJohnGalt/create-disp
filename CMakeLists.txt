cmake_minimum_required(VERSION 3.10)

project(create_disp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(LIBDRM REQUIRED libdrm)
pkg_check_modules(LIBSYSTEMD REQUIRED libsystemd)

file(GLOB SOURCES "src/*.cpp")

add_executable(create-disp ${SOURCES})

include_directories(
    ${LIBDRM_INCLUDE_DIRS}
    ${LIBSYSTEMD_INCLUDE_DIRS}
)

link_directories(
    ${LIBDRM_LIBRARY_DIRS}
    ${LIBSYSTEMD_LIBRARY_DIRS}
)

target_include_directories(create-disp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/include/libdrm
    /usr/include/android
)

target_link_libraries(create-disp PRIVATE
    Threads::Threads
    hwc2
    hybris-common
    hybris-eglplatformcommon
    hybris-platformcommon
    gralloc
    ${LIBDRM_LIBRARIES}
    ${LIBSYSTEMD_LIBRARIES}
)
