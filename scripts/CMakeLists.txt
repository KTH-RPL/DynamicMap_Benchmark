cmake_minimum_required(VERSION 3.14)
project(dynamic_benchmark
	VERSION 1.0.0
	DESCRIPTION "DynamicMap benchmark tools, including data processing, evaluation"
	LANGUAGES CXX
)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680#issuecomment-700642602
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
set(LOG_LIB glog::glog gflags)
find_package(PCL REQUIRED)

function(create_executable target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} ${PCL_LIBRARIES} ${LOG_LIB})
endfunction()

create_executable(create_rawmap cpp/create_rawmap.cpp)
create_executable(extract_gtcloud cpp/extract_gtcloud.cpp)
create_executable(export_eval_pcd cpp/export_eval_pcd.cpp)
