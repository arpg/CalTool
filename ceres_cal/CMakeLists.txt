project(ceres_cal)
cmake_minimum_required( VERSION 2.8 )
# Add to module path, so we can find our cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)



include(def_library)
include(export_package)
include(def_executable)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -Wall -Wextra -Wno-unused-parameter")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wno-unused-parameter")



find_package(Ceres REQUIRED)
find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)
find_package(HAL REQUIRED)

include_directories(${HAL_INCLUDE_DIRS}
		    ${Ceres_INCLUDE_DIRS}
		    ${Eigen_INCLUDE_DIRS}
		    ${Sophus_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR})

def_executable(ceres_cal
  SOURCES ceres_cal.cpp CSVReader.cpp
  LINK_LIBS
  ${HAL_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${CERES_LIBRARIES}
  ${CMAKE_DL_LIBS}
  )

def_executable(test_csv
  SOURCES test_csv.cpp CSVReader.cpp
  )
