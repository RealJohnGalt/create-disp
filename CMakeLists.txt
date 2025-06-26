cmake_minimum_required(VERSION 3.10)

project(create_disp)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(create-disp create-disp.cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBDRM REQUIRED libdrm)

include_directories(${LIBDRM_INCLUDE_DIRS})
link_directories(${LIBDRM_LIBRARY_DIRS})

target_include_directories(create-disp PRIVATE
    /usr/include/libdrm
    /usr/include/android
)

target_link_libraries(create-disp
    hwc2
    hybris-common
    hybris-eglplatformcommon
    hybris-platformcommon
    gralloc
    ${LIBDRM_LIBRARIES}
)
