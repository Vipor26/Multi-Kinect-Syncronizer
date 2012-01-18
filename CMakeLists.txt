cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(multiple_kinects)

#options
option(BUILD_EXAMPLES "Build all of the multiple kinects examples." ON)
option(BUILD_TESTS "Build all of the multiple kinects tests (not gtests) )." ON)
# Include everthing related to PCL
find_package(PCL 1.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Changed output path to bin and lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Add default source files
set(        MANAGER_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/multiGrabManager/multiGrabManager.cpp)
list(APPEND MANAGER_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/multiGrabManager/synchronizer.cpp)
list(APPEND MANAGER_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/multiGrabManager/labeledCloud.cpp)
list(APPEND MANAGER_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/multiGrabManager/camera.cpp)

# Add default header files
set(        MANAGER_LIB_HEADERS ${PROJECT_SOURCE_DIR}/include/multiGrabManager/multiGrabManager.h)
list(APPEND MANAGER_LIB_HEADERS ${PROJECT_SOURCE_DIR}/include/multiGrabManager/synchronizer.h)
list(APPEND MANAGER_LIB_HEADERS ${PROJECT_SOURCE_DIR}/include/multiGrabManager/labeledCloud.h)
list(APPEND MANAGER_LIB_HEADERS ${PROJECT_SOURCE_DIR}/include/multiGrabManager/camera.h)

include_directories(${PROJECT_SOURCE_DIR}/include/multiGrabManager)

# Compile the libsegway Library
add_library(multiGrabManager ${MANAGER_LIB_SRCS} ${MANAGER_LIB_HEADERS})
target_link_libraries(multiGrabManager ${PCL_LIBRARIES})

IF(BUILD_EXAMPLES)
    # Compile the Example program
    add_executable(multipleKinect ${PROJECT_SOURCE_DIR}/src/examples/multipleKinect.cpp)
    # Link the Test program to the MultipleKinect library
    target_link_libraries(multipleKinect multiGrabManager)
    target_link_libraries(multipleKinect ${PCL_LIBRARIES})
    
    #compile the test program
    add_executable(principleTest ${PROJECT_SOURCE_DIR}/src/examples/principleDemo.cpp)
    target_link_libraries(principleTest ${PCL_LIBRARIES})

    add_executable(simpleDemo ${PROJECT_SOURCE_DIR}/src/examples/simpleDemo.cpp
                              ${PROJECT_SOURCE_DIR}/src/multiGrabManager/synchronizer.cpp)
    target_link_libraries(simpleDemo ${PCL_LIBRARIES})
ENDIF(BUILD_EXAMPLES)

IF(BUILD_TESTS)
    add_executable(singleCameraViewer ${PROJECT_SOURCE_DIR}/test/singleCameraViewer.cpp)
    target_link_libraries(singleCameraViewer ${PCL_LIBRARIES})

    add_executable(multiCameraViewer ${PROJECT_SOURCE_DIR}/test/multiCameraViewer.cpp)
    target_link_libraries(multiCameraViewer ${PCL_LIBRARIES})
ENDIF(BUILD_TESTS)


