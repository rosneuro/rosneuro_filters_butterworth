cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_filters_butterworth)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(ROSNEURO_DATA_MIN_VERSION "1.0.0")

##############################################################################
# Find dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 roslib
			 std_msgs
			 pluginlib
			 rosneuro_filters)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(RTF_LIBRARIES rtfilter)
find_path(RTF_INCLUDE_DIRS rtfilter.h)

find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
					${RTF_INCLUDE_DIRS}
					include
                    include/${PROJECT_NAME}/)


##############################################################################
# Define package
##############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  ${PROJECT_NAME}
	
  CATKIN_DEPENDS
  	roscpp
	roslib
	std_msgs
	pluginlib
	rosneuro_filters
  DEPENDS
)


##############################################################################
# Build
##############################################################################

add_definitions(${EIGEN3_DEFINITIONS})

# Butterworth frequency filter
add_library(${PROJECT_NAME} src/Butterworth.cpp)
target_link_libraries(${PROJECT_NAME} 	${Eigen3_LIBRARIES} 
										${RTF_LIBRARIES} 
										${catkin_LIBRARIES}
)


#################
## Executables ##
#################


catkin_add_gtest(TestButterworth test/test_butterworth.cpp)
target_link_libraries(TestButterworth ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestButterworth ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
include_directories(${GTEST_INCLUDE_DIRS} gmock pthread include include/${PROJECT_NAME}/ yaml-cpp)

add_executable(butterworth_simloop test/butterworth_simloop.cpp)
target_link_libraries(butterworth_simloop ${PROJECT_NAME} ${catkin_LIBRARIES}) 

add_executable(butterworth_simloop_config test/butterworth_simloop_config.cpp)
target_link_libraries(butterworth_simloop_config ${PROJECT_NAME} ${catkin_LIBRARIES}) 

add_executable(butterworth_simloop_chain test/butterworth_simloop_chain.cpp)
target_link_libraries(butterworth_simloop_chain ${PROJECT_NAME} ${catkin_LIBRARIES}) 


#################
##   Install   ##
#################
install(TARGETS ${PROJECT_NAME}

	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES plugins_rosneuro_filters_butterworth.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
