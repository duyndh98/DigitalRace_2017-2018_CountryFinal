cmake_minimum_required(VERSION 2.8)

project( radon )
find_package( OpenCV REQUIRED )

set( source_files
	Radon.h
	Radon.cpp
	Singleton.h
)
set(CMAKE_CXX_FLAGS "-fPIC ${CFLAGS_WARN} -std=c++11 ${CMAKE_CXX_FLAGS}")
set( lib_name radon)
add_library( ${lib_name} ${source_files} )


set( test_exec_name test_radon)
add_executable( ${test_exec_name}  test_radon.cpp )

target_link_libraries( ${test_exec_name} ${lib_name} ${OpenCV_LIBS})
