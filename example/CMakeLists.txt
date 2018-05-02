cmake_minimum_required(VERSION 2.8)

project( CarControl-0.3 )

find_package( OpenCV REQUIRED )
set( OPENNI2_INCLUDE /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Include)
set( OPENNI2_REDIST  /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Redist)
include_directories( ${OPENNI2_INCLUDE} )

link_directories ( ${OPENNI2_REDIST} )


if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
               STRINGS "Debug" "Release")
endif()
IF(MSVC)
  ADD_DEFINITIONS(-DNOMINMAX)
  # Another MSVC quirk. http://stackoverflow.com/questions/6832666/lnk2019-when-including-asio-headers-solution-generated-with-cmake
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP") # For multi-threaded compilation on MSVC
ELSE()
  IF(APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++") # C++11 support on Apple clang
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# C++11 support on GCC
  ENDIF()
ENDIF()

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

include_directories(
      ${OpenCV_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}/stereo_vision
      ${CMAKE_CURRENT_SOURCE_DIR}/lane_detection
      ${CMAKE_CURRENT_SOURCE_DIR}/lane_detection/msac
      ${CMAKE_CURRENT_SOURCE_DIR}/peripheral_driver/i2c
      ${CMAKE_CURRENT_SOURCE_DIR}/peripheral_driver/uart
      ${CMAKE_CURRENT_SOURCE_DIR}/HAL	
      ${CMAKE_CURRENT_SOURCE_DIR}/multilane
      ${CMAKE_CURRENT_SOURCE_DIR}/radon
      ${CMAKE_CURRENT_SOURCE_DIR}/extract_info
	    ${CMAKE_CURRENT_SOURCE_DIR}/openni2
)

link_directories( ${LIB_FREENECT_INSTALL_DIR}/lib
                  ${LIBRARY_OUTPUT_PATH}
)

find_package(OpenMP)
if(OPENMP_FOUND)
    message("Find OpenMP: ${OpenMP_C_FLAGS}, ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(stereo_vision)
add_subdirectory(lane_detection)
add_subdirectory(peripheral_driver)
add_subdirectory(HAL)
add_subdirectory(multilane)
add_subdirectory(radon)
add_subdirectory(extract_info)
add_subdirectory(openni2)

set( exec_name test-autocar)
set( source_files
    main_pid/automaticCar.cpp
    main_pid/image_processing.cpp
    main_pid/lane_detection.cpp
    main_pid/sign.cpp
    main_pid/control.cpp
    # if you write extra libraries for automaticCar.cpp, declare them here.
    # See CMakeLists.txt in sub-directory for more details.
)
add_executable( ${exec_name} ${source_files} )
target_link_libraries( ${exec_name} vanishing-point i2c-pwm uart kinect-cv2 OpenNI2 HAL multilane ${OpenCV_LIBS})

add_executable( threshold-selection 
main_pid/thresholdSelection.cpp
main_pid/camera.h
main_pid/my_assert.h
main_pid/real_camera.h
main_pid/real_camera.cpp)
target_link_libraries( threshold-selection vanishing-point i2c-pwm uart kinect-cv2 OpenNI2 HAL multilane ${OpenCV_LIB})

add_executable( threshold-Depth 
main_pid/thresholdDepth.cpp
main_pid/camera.h
main_pid/my_assert.h
main_pid/real_camera.h
main_pid/real_camera.cpp)
target_link_libraries( threshold-Depth vanishing-point i2c-pwm uart kinect-cv2 OpenNI2 HAL multilane ${OpenCV_LIB})
