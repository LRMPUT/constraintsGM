# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# Example Project configuration
PROJECT(CONTRAINTS_GM)

SET(CMAKE_CXX_FLAGS "${CXX_STANDARD} -Wall -Wextra -Wshadow -Wconversion -pthread")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/resources")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")
option(LOG_DEBUG "Turn on debugging log" OFF)
mark_as_advanced(LOG_DEBUG)
if(LOG_DEBUG)
    add_definitions(-DLOG_DEBUG)
endif(LOG_DEBUG)

# core libraries
option(BUILD_DEFS "Build Defs library" ON)
mark_as_advanced(BUILD_DEFS)
option(BUILD_VISUALIZER "Build Visualizer" ON)
mark_as_advanced(BUILD_VISUALIZER)
option(BUILD_ROBOTMODEL_MAT "Build mathematical robot model library" ON)
mark_as_advanced(BUILD_ROBOTMODEL_MAT)
option(BUILD_OPTIMIZER "Build black box optimizer library" ON)
mark_as_advanced(BUILD_OPTIMIZER)
option(BUILD_UTILITIES "Build Utilities library" ON)
mark_as_advanced(BUILD_UTILITIES)
option(BUILD_COLLISION_DETECTION "Build collision detection module" ON)
mark_as_advanced(BUILD_COLLISION_DETECTION)
option(BUILD_WITH_FCL "Build with Fast Collision Detection" ON)
mark_as_advanced(BUILD_WITH_FCL)
option(BUILD_REGRESSION "Build regression module" ON)
mark_as_advanced(BUILD_REGRESSSION)

# demos
option(BUILD_CONTRAINTS_GM_DEMO "Build CONTRAINTS_GM demo" ON)
mark_as_advanced(BUILD_CONTRAINTS_GM_DEMO)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

if(BUILD_VISUALIZER)
    add_definitions(-DBUILD_VISUALIZER)
endif(BUILD_VISUALIZER)

if(BUILD_WITH_FCL)
    add_definitions(-DBUILD_WITH_FCL)
endif(BUILD_WITH_FCL)

#additional dependencies
# Boost
# FIND_PACKAGE(Boost COMPONENTS system filesystem thread asio)

#Eigen
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/3rdParty/Eigen)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

#NatNet (3rdParty)
INCLUDE_DIRECTORIES ( "${CMAKE_SOURCE_DIR}/3rdParty" )

if(BUILD_WITH_FCL)
    find_package(CCD REQUIRED)
    include_directories(${CCD_INCLUDE_DIRS})
    LINK_DIRECTORIES(${CCD_BINARY_DIR})

    find_package(FCL REQUIRED)
    include_directories(${FCL_INCLUDE_DIR})
    include_directories(${FCL_INCLUDE_DIR2})
    LINK_DIRECTORIES(${FCL_BINARY_DIR})
endif(BUILD_WITH_FCL)

if(BUILD_VISUALIZER)

    #Qt5
    find_package(Qt5Widgets REQUIRED)
    find_package(Qt5Core REQUIRED)
    find_package(Qt5Xml REQUIRED)
    find_package(Qt5OpenGL REQUIRED)
    MESSAGE("widgets: " ${Qt5Core_LIBRARIES})
    include_directories(${Qt5Widgets_INCLUDE_DIRS})
    include_directories(${Qt5Core_INCLUDE_DIRS})
    include_directories(${Qt5Xml_INCLUDE_DIRS})
    include_directories(${Qt5OpenGL_INCLUDE_DIRS})

    #required by Qt5
    # Use "-fPIC" / "-fPIE" for all targets by default, including static libs
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    # CMake doesn't add "-pie" by default for executables (CMake issue #14983)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fPIC")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    SET(CMAKE_CACHEFILE_DIR_FLAGS "${CMAKE_C_FLAGS} -fPIC")

    FIND_PACKAGE(OpenGL)
    include_directories(${OPENGL_INCLUDE_DIR})
    link_directories(${OPENGL_LIBRARY})

    #QGLViewer library
    FIND_PACKAGE(QGLViewer)
    include_directories(${QGLVIEWER_INCLUDE_DIR})
#    include_directories(${QT_INCLUDES})
    link_directories(${QGLVIEWER_LIBRARY})

endif(BUILD_VISUALIZER)

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
        SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
        SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

if(MSVC)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR CMake)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# Create the constraintsGMConfig.cmake
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
file(RELATIVE_PATH REL_LIB_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_LIB_DIR}")

set(CONF_INCLUDE_DIRS "${RELATIVE_PATH}" "${CMAKE_SOURCE_DIR}")

set(CONF_LIB_DIRS "${RELATIVE_PATH}" "/build/lib")

configure_file(constraintsGMConfig.cmake.in
  "${CMAKE_SOURCE_DIR}/constraintsGMConfig.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "${CONF_INCLUDE_DIRS}/include")
set(CONF_LIB_DIRS "${CMAKE_SOURCE_DIR}/build/lib")

configure_file(constraintsGMConfig.cmake.in
  "${CMAKE_SOURCE_DIR}/constraintsGMConfig.cmake" @ONLY)

# Install the constraintsGMConfig.cmake
install(FILES
  "${CMAKE_SOURCE_DIR}/constraintsGMConfig.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)


###############################################################################
#
# SIMULATOR tinyXML2 library
#
###############################################################################

#add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)
add_library(tinyxml2 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)

###############################################################################
#
# SIMULATOR Math Robot Model library
#
###############################################################################

if(BUILD_ROBOTMODEL_MAT)
        file(GLOB ROBOTMODELMAT_SOURCES
            "${CMAKE_SOURCE_DIR}/src/RobotModelMat/*.cpp"
        )
        file(GLOB ROBOTMODELMAT_HEADERS
            "${CMAKE_SOURCE_DIR}/include/RobotModelMat/*.h"
        )

        ADD_LIBRARY(RobotModelMat STATIC ${ROBOTMODELMAT_SOURCES} ${ROBOTMODELMAT_HEADERS})
        TARGET_LINK_LIBRARIES(RobotModelMat Regression Optimizer Defs CollisionDetection)
#        INSTALL(TARGETS Controller RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${CONTROLLER_HEADERS} DESTINATION include/walkers/Utilities/)
endif(BUILD_ROBOTMODEL_MAT)

###############################################################################
#
# WALKERS Utilities library
#
###############################################################################

if(BUILD_UTILITIES)
        file(GLOB UTILITIES_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Utilities/*.cpp"
        )
        file(GLOB UTILITIES_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Utilities/*.h"
        )
        add_library(Utilities STATIC ${UTILITIES_SOURCES} ${UTILITIES_HEADERS})
        TARGET_LINK_LIBRARIES(Utilities Defs ${OPENGL_LIBRARY} glut)
#        if(BUILD_VISUALIZER)
#            TARGET_LINK_LIBRARIES(Utilities ${OPENGL_LIBRARY} glut)
#        endif(BUILD_VISUALIZER)
        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${TRACKER_HEADERS} DESTINATION include/walkers/Utilities/)
endif(BUILD_UTILITIES)

###############################################################################
#
# Defs library
#
###############################################################################

if(BUILD_DEFS)
        file(GLOB DEFS_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Defs/*.cpp"
        )
        file(GLOB DEFS_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Defs/*.h"
        )
        ADD_LIBRARY(Defs STATIC ${DEFS_SOURCES} ${DEFS_HEADERS})
        INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/Defs/)
endif(BUILD_DEFS)

###############################################################################
#
# SIMULATOR Visualizer library
#
###############################################################################

if(BUILD_VISUALIZER)
        file(GLOB VISUALIZER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Visualizer/*.cpp"
        )
        file(GLOB VISUALIZER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Visualizer/*.h"
        )
        ADD_LIBRARY(Visualizer STATIC ${VISUALIZER_SOURCES} ${VISUALIZER_HEADERS})
        TARGET_LINK_LIBRARIES(Visualizer tinyxml2 ${QGLVIEWER_LIBRARY} GL GLU glut QGLViewer-qt5 Qt5Core Qt5Widgets Qt5Xml Qt5OpenGL Qt5Gui CollisionDetection RobotModelMat)
endif(BUILD_VISUALIZER)

###############################################################################
#
# Optimizer library
#
###############################################################################

if(BUILD_OPTIMIZER)
        file(GLOB OPTIMIZER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Optimizer/*.cpp"
        )
        file(GLOB OPTIMIZER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Optimizer/*.h"
        )
        ADD_LIBRARY(Optimizer STATIC ${OPTIMIZER_SOURCES} ${OPTIMIZER_HEADERS})
        TARGET_LINK_LIBRARIES(Optimizer tinyxml2)
endif(BUILD_OPTIMIZER)

###############################################################################
#
# CONTRAINTS_GM Utilities library
#
###############################################################################

if(BUILD_CONTRAINTS_GM_UTILITIES)
        file(GLOB UTILITIES_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Utilities/*.cpp"
        )
        file(GLOB UTILITIES_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Utilities/*.h"
        )
        add_library(Utilities STATIC ${UTILITIES_SOURCES} ${UTILITIES_HEADERS})
        TARGET_LINK_LIBRARIES(Utilities Defs)
        if(BUILD_VISUALIZER)
            TARGET_LINK_LIBRARIES(Utilities ${OPENGL_LIBRARY} glut)
        endif(BUILD_VISUALIZER)
        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${TRACKER_HEADERS} DESTINATION include/walkers/Utilities/)
endif(BUILD_CONTRAINTS_GM_UTILITIES)

###############################################################################
#
# Collision detection library
#
###############################################################################

if(BUILD_COLLISION_DETECTION)
        file(GLOB COLLISION_DETECTION_SOURCES
            "${CMAKE_SOURCE_DIR}/src/CollisionDetection/*.cpp"
        )
        file(GLOB COLLISION_DETECTION_HEADERS
            "${CMAKE_SOURCE_DIR}/include/CollisionDetection/*.h"
        )
        if(NOT BUILD_WITH_FCL)
            list(REMOVE_ITEM COLLISION_DETECTION_SOURCES ${CMAKE_SOURCE_DIR}/src/CollisionDetection/CollisionDetectionFCL.cpp)
            list(REMOVE_ITEM COLLISION_DETECTION_HEADERS ${CMAKE_SOURCE_DIR}/include/CollisionDetection/CollisionDetectionFCL.h)
        endif(NOT BUILD_WITH_FCL)
        ADD_LIBRARY(CollisionDetection STATIC ${COLLISION_DETECTION_SOURCES} ${COLLISION_DETECTION_HEADERS})
        TARGET_LINK_LIBRARIES(CollisionDetection RobotModelMat coldet tinyxml2 Defs)
        if(BUILD_WITH_FCL)
            TARGET_LINK_LIBRARIES(CollisionDetection ${FCL_LIBRARY} ${CCD_LIBRARY})
        endif(BUILD_WITH_FCL)
#        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${VISUALIZER_HEADERS} DESTINATION include/walkers/Visualizer/)
endif(BUILD_COLLISION_DETECTION)

###############################################################################
#
# Regression library
#
###############################################################################

if(BUILD_REGRESSION)
        file(GLOB REGRESSION_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Regression/*.cpp"
        )
        file(GLOB REGRESSION_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Regression/*.h"
        )
        ADD_LIBRARY(Regression STATIC ${REGRESSION_SOURCES} ${REGRESSION_HEADERS})
        TARGET_LINK_LIBRARIES(Regression Optimizer Utilities tinyxml2 Defs)
#        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${VISUALIZER_HEADERS} DESTINATION include/walkers/Visualizer/)
endif(BUILD_REGRESSION)

###############################################################################
#
# CONTRAINTS_GM demo
#
###############################################################################

if(BUILD_CONTRAINTS_GM_DEMO)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./main.cpp)
        ADD_EXECUTABLE(robotModelGM ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(robotModelGM  Regression CollisionDetection ${CCD_LIBRARY} RobotModelMat Utilities tinyxml2 Defs boost_system)
        INSTALL(TARGETS robotModelGM RUNTIME DESTINATION bin)
endif(BUILD_CONTRAINTS_GM_DEMO)

###############################################################################
#
# Collision detection demo
#
###############################################################################

if(BUILD_COLDET_DEMO)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoColdet.cpp)
        ADD_EXECUTABLE(demoColdet ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoColdet CollisionDetection Utilities tinyxml2 Defs boost_system ${OpenCV_LIBS})
        INSTALL(TARGETS demoColdet RUNTIME DESTINATION bin)

endif(BUILD_COLDET_DEMO)

###############################################################################
#
# Gaussian Mixture model of the robot
#
###############################################################################

if(BUILD_COLDET_DEMO)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/robotGMmodel.cpp)
        ADD_EXECUTABLE(robotGMmodel ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(robotGMmodel Regression CollisionDetection ${CCD_LIBRARY} RobotModelMat Utilities tinyxml2 Defs boost_system ${OpenCV_LIBS})
        INSTALL(TARGETS demoColdet RUNTIME DESTINATION bin)

        SET(DEMO_SOURCES ./demos/robotGMCollision.cpp)
        ADD_EXECUTABLE(robotGMCollision ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(robotGMCollision Regression CollisionDetection ${CCD_LIBRARY} RobotModelMat Utilities ${OpenCV_LIBS} tinyxml2 Defs boost_system)

        SET(DEMO_SOURCES ./demos/robotGMOutWorkspaceModel.cpp)
        ADD_EXECUTABLE(robotGMOutWorkspaceModel ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(robotGMOutWorkspaceModel Regression CollisionDetection ${CCD_LIBRARY} RobotModelMat Utilities ${OpenCV_LIBS} tinyxml2 Defs boost_system)
endif(BUILD_COLDET_DEMO)

###############################################################################
#
# SIMULATOR demo Visualizer
#
###############################################################################

if(BUILD_VISUALIZER)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoVisualizer.cpp)
        ADD_EXECUTABLE(demoVisualizer ${DEMO_SOURCES})
        target_compile_options(demoVisualizer PRIVATE -fPIC)
        TARGET_LINK_LIBRARIES(demoVisualizer Visualizer Defs RobotModelMat tinyxml2 GL GLU glut QGLViewer-qt5 Qt5Core Qt5Widgets Qt5Xml Qt5OpenGL Qt5Gui)
        if(BUILD_WITH_FCL)
            TARGET_LINK_LIBRARIES(demoVisualizer ${FCL_LIBRARY} ${CCD_LIBRARY})
        endif(BUILD_WITH_FCL)
        INSTALL(TARGETS demoVisualizer RUNTIME DESTINATION bin)

endif(BUILD_VISUALIZER)
