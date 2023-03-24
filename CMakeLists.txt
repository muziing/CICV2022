cmake_minimum_required(VERSION 3.24)
project(CICV2022)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)
include_directories(include/panoSimApi)
include_directories(src/Sample)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(CICV2022
        include/panoSimApi/BasicsBusDef.h
        include/panoSimApi/BusAccessor.h
        include/panoSimApi/Export.h
        include/panoSimApi/Import.h
        include/panoSimApi/Interface.h
        include/panoSimApi/PanoSimApi.h
        include/panoSimApi/SensorBusDef.h
        src/Sample/control.cc
        src/Sample/control.h
        src/Sample/lqr_control.cc
        src/Sample/lqr_control.h
        src/Sample/pure_puresuit_control.cc
        src/Sample/pure_puresuit_control.h
        src/Sample/reference_line.cc
        src/Sample/reference_line.h
        src/Sample/resource.h
        src/Sample/Sample.cc)
