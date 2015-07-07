cmake_minimum_required(VERSION 2.8.3)
project(homography_calc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cv_bridge
  message_generation
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# There's no version hint in the yaml-cpp headers, so get the version number
# from pkg-config.
find_package(PkgConfig REQUIRED)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")

add_subdirectory(database_interface)

## Generate messages in the 'msg' folder
add_message_files(FILES 
	matchedPoints.msg 
	features.msg 
	decomposition.msg
	controlLawInputs.msg
	controlLawOutputs.msg
)

#add_service_files(
#	FILES
#	saveKeyframe.srv
#	getKeyframe.srv
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  homography_calc
)

catkin_package(
	INCLUDE_DIRS include database_interface/include
	LIBRARIES postgresql_database
	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "include"
  "eigen"
  database_interface/include
)

#SET(DB_SRC ${CMAKE_SOURCE_DIR}/homography_calc/include/db_test.cpp)
#ADD_LIBRARY(DB_LIBS ${DB_SRC} )

SET(BASIC_LIBS ${catkin_LIBRARIES} ${OpenCV_LIBS} opencv_nonfree postgresql_database)
# Libraries for catkin, OpenCV, SURF features, and the SQL database.

#####################
## Libs for homest ##
#####################
set(HOMEST_SRC_DIR /usr/local/include/homest-1.4)  # Set HOMEST_SRC_DIR as a variable
SET(LEVMAR_INCDIR "/usr/local/include/levmar-2.6" CACHE PATH "Path to LEVMAR library header")
SET(LAPACKBLAS_DIR "/usr/lib" CACHE PATH "Path to lapack/blas libraries") # Some deep magic of CMake to find the lapack library 
SET(LAPACKBLAS_LIB_NAMES "lapack;blas" CACHE STRING "The name of the lapack & blas libraries")

INCLUDE_DIRECTORIES(${LEVMAR_INCDIR})  # Need to include Levmar library
INCLUDE_DIRECTORIES(${HOMEST_SRC_DIR}) # Include this directory
INCLUDE_DIRECTORIES(${LAPACKBLAS_DIR})  # Include the Lapack library as well. 

ADD_LIBRARY(homest STATIC  # Add a bunch of source and header files to a library
  ${HOMEST_SRC_DIR}/calc_2Dhomog_coeffs.c ${HOMEST_SRC_DIR}/homest.c 
  ${HOMEST_SRC_DIR}/lqs.c ${HOMEST_SRC_DIR}/linalg.c ${HOMEST_SRC_DIR}/norm.c
  ${HOMEST_SRC_DIR}/buckets.c ${HOMEST_SRC_DIR}/compiler.h ${HOMEST_SRC_DIR}/homest.h 
  ${HOMEST_SRC_DIR}/lqs.h ${HOMEST_SRC_DIR}/maplefuncs.h ${HOMEST_SRC_DIR}/util.h
)
SET(HOMEST_LIBS homest levmar m ${LAPACKBLAS_LIB_NAMES})  # Add libraries that will be linked to files.
#############
## Testing ##
#############

# Include the eigen directory for all executables so they can
# use the eigen-type matrix structures. 
#include_directories(include )
#include_directories("eigen")

### add_dependencies is important so that the files can link into the generated messages (i.e. that the messages are generated ahead of time.) 

# Old, deprecated version 1.0 for finding the homography. Do not use. 
#add_executable(old_homog src/old_homog.cpp)
#target_link_libraries(old_homog ${catkin_LIBRARIES} ${OpenCV_LIBS} opencv_nonfree)

# Subscribe to an image topic and extract the features (SURF), then publish the raw 
# features. They will need to be matched in matchRawFeatures. 
add_executable(findFeatures src/findFeatures.cpp)
target_link_libraries(findFeatures ${BASIC_LIBS})
add_dependencies(findFeatures homography_calc_generate_messages_cpp)

# Match features that have previously been found
add_executable(matchRawFeatures src/matchRawFeatures.cpp)
target_link_libraries(matchRawFeatures ${BASIC_LIBS})
add_dependencies(matchRawFeatures homography_calc_generate_messages_cpp)

# A compound node that performs the function of both findFeatures and matchRawFeatures.
# Use in an either-or configuration. 
add_executable(findAndMatch src/findFeaturesAndMatch.cpp )
target_link_libraries(findAndMatch ${BASIC_LIBS} )
add_dependencies(findAndMatch homography_calc_generate_messages_cpp)

# Test code for various things, used once to figure out where a bottleneck was. 
#add_executable(speedup src/speedup.cpp)
#target_link_libraries(speedup ${BASIC_LIBS})

# Takes in matched feature points from matchRawFeatures or findAndMatch and 
# computes the homography using OpenCV's findHomography. 
add_executable(cvHomog src/cvHomog.cpp)
target_link_libraries(cvHomog ${BASIC_LIBS})
add_dependencies(cvHomog homography_calc_generate_messages_cpp)

# Takes in matched feature points from matchRawFeatures or findAndMatch and 
# computes the homography using the homest library 
add_executable(homestHomog src/homestHomog.cpp)
target_link_libraries(homestHomog ${BASIC_LIBS} ${HOMEST_LIBS})
add_dependencies(homestHomog homography_calc_generate_messages_cpp)

# Input is a 3x3 homography. This code decomposes the homography using the algorithm in 
# "A Deeper Understanding of Homography." Publishes a custom message with both sets
# of decompositions.
add_executable(decompose src/homogDecompose.cpp)
target_link_libraries(decompose ${BASIC_LIBS})
add_dependencies(decompose homography_calc_generate_messages_cpp)

# Control law based solely on the homography (no decomposition.) Publishes a translation
# and a rotation (in cross vector form)
add_executable(homogControlLaw src/homogControlLaw.cpp)
target_link_libraries(homogControlLaw ${BASIC_LIBS} )
add_dependencies(homogControlLaw homography_calc_generate_messages_cpp)

# Homography filter based on pixel intensities. 
#ADD_LIBRARY(ESMlib STATIC ${CMAKE_SOURCE_DIR}/homography_calc/libs/ESMlib.a ) 
#SET_TARGET_PROPERTIES(ESMlib PROPERTIES LINKER_LANGUAGE C)

add_executable(ESMHomographyFilter src/ESMHomographyFilter.cpp)
target_link_libraries(ESMHomographyFilter ${BASIC_LIBS} )
add_dependencies(ESMHomographyFilter homography_calc_generate_messages_cpp)
# The following had to be linked in after the other things, I think.
target_link_libraries(ESMHomographyFilter ${CMAKE_SOURCE_DIR}/homography_calc/libs/ESMlib.a)

#add_executable(database include/db_test.cpp)
#target_link_libraries(database ${BASIC_LIBS} )
#target_link_libraries(database postgresql_database)
#add_dependencies(database homography_calc_generate_messages_cpp)

add_executable(dbTest funcTesting/db_testing.cpp)
target_link_libraries(dbTest ${BASIC_LIBS} )
add_dependencies(dbTest homography_calc_generate_messages_cpp)

