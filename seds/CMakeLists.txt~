cmake_minimum_required(VERSION 2.8.3)#SEDS2.4.6
project(seds)

set(CMAKE_CXX_FLAGS "-std=c++98 ${CMAKE_CXX_FLAGS}")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#common commands for building c++ executables and libraries
FILE(GLOB theSourceFiles src/*.cpp)
FILE(GLOB theHeaderFiles include/*.h)

find_package(catkin REQUIRED COMPONENTS 
		roscpp
		std_msgs
#		message_generation		
		mathlib
)


catkin_package(	
	INCLUDE_DIRS include	
	LIBRARIES ${PROJECT_NAME}	
	CATKIN_DEPENDS roscpp mathlib# message_runtime #pass the dependencies to the package in find_package
)

## Specify additional locations of header files

include_directories(include ${catkin_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(${PROJECT_NAME}
${theSourceFiles} 
${theHeaderFiles}
)

#install(DIRECTORY include/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

### Declare a cpp executable
#add_executable(${PROJECT_NAME}
#		${theSourceFiles})

### Add dependence to the iiwa_msg module for the executable
#add_dependencies(${PROJECT_NAME}-example				)
				
### Add dependence to the iiwa_msg module for the library
#add_dependencies(${PROJECT_NAME}				)
						
# Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib／libmathlib.so)

#install(DIRECTORY include/       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
