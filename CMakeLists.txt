project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp
            src/utils.h
            src/car.h
            src/navigation.h
            src/map.h
            src/path.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if(UNIX)
include_directories(/devsystem/dependencies/ubuntu16/openssl/libopenssl-1.1.0e_x86_64/include)
link_directories(/devsystem/dependencies/ubuntu16/openssl/libopenssl-1.1.0e_x86_64/lib)
endif(UNIX) 

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
