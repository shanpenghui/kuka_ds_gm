cmake_minimum_required(VERSION 2.8.3)
project(iiwa_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
FILE(GLOB theSourceFiles_math ~/kuka_catkin/src/mathlib/src/*.cpp)
FILE(GLOB theHeaderFiles_math ~/kuka_catkin/src/mathlib/include/*.h)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  tf
  cmake_modules
  seds
  mathlib
  message_filters 
  geometry_msgs
)

#add eigen
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
######

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME} 
	CATKIN_DEPENDS  iiwa_msgs seds mathlib message_filters geometry_msgs
) 


#link_directories(/home/edward/kuka_catkin/devel/libs)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS} 
)


## Declare a cpp library
add_library(${PROJECT_NAME}
	include/iiwaRos.h
	src/iiwaRos.cpp
#	include/Myadd.h
#	src/Myadd.cpp
        #~/kuka_catkin/src/seds/include/GMR.h      
)

## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME}
				iiwa_msgs_generate_messages_cpp)


##########################################add file##############################################################

#####add the readGMM test
add_executable(readGMM src/SEDS_GMM/readGMM.cpp )
target_link_libraries(readGMM ${catkin_LIBRARIES} )
add_dependencies(readGMM iiwa_stack-master-Sunrise1.9_generate_messages_cpp) 

add_executable(readGMM1 src/SEDS_GMM/readGMM1.cpp )
target_link_libraries(readGMM1 ${catkin_LIBRARIES} )
add_dependencies(readGMM1 iiwa_stack-master-Sunrise1.9_generate_messages_cpp) 

add_executable(pt_sj_go_SEDS src/SEDS_GMM/pt_sj_go_SEDS.cpp )
target_link_libraries(pt_sj_go_SEDS ${catkin_LIBRARIES} )
add_dependencies(pt_sj_go_SEDS iiwa_stack-master-Sunrise1.9_generate_messages_cpp) 

add_executable(pt_sj_go_change src/SEDS_GMM/pt_sj_go_change.cpp )
target_link_libraries(pt_sj_go_change ${catkin_LIBRARIES} )
add_dependencies(pt_sj_go_change iiwa_stack-master-Sunrise1.9_generate_messages_cpp) 

add_executable(get_all_sensor_in_one src/topic_synchronizer/get_all_sensor_in_one.cpp )
target_link_libraries(get_all_sensor_in_one ${catkin_LIBRARIES} )
add_dependencies(get_all_sensor_in_one iiwa_stack-master-Sunrise1.9_generate_messages_cpp) 

add_executable(joint_position_for_gazebo src/simulate_in_gazebo/joint_position_for_gazebo.cpp )
target_link_libraries(joint_position_for_gazebo ${catkin_LIBRARIES} )
add_dependencies(joint_position_for_gazebo iiwa_stack-master-Sunrise1.9_generate_messages_cpp) 

add_executable(pt_sj_go_SEDS_for_gazebo src/simulate_in_gazebo/pt_sj_go_SEDS_for_gazebo.cpp )
target_link_libraries(pt_sj_go_SEDS_for_gazebo ${catkin_LIBRARIES} )
add_dependencies(pt_sj_go_SEDS_for_gazebo iiwa_stack-master-Sunrise1.9_generate_messages_cpp) 

########################################################################################################################
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


######## Declare a cpp executable
######add_executable(${PROJECT_NAME}-example
######    src/main.cpp
#######	src/Myadd.cpp
######)

######## Add dependence to the iiwa_msg module for the executable
######add_dependencies(${PROJECT_NAME}-example
######				iiwa_msgs_generate_messages_cpp)
######							
######## Specify libraries to link a library or executable target against
######target_link_libraries(${PROJECT_NAME}-example
######    ${PROJECT_NAME}
######    ${catkin_LIBRARIES}
######)













##all cpp file
#add_executable(pt_listener src/pt_listener.cpp)
#target_link_libraries(pt_listener ${catkin_LIBRARIES})
#add_dependencies(pt_listener iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_start src/pt_start.cpp)
#target_link_libraries(pt_start ${catkin_LIBRARIES})
#add_dependencies(pt_start iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_client src/pt_client.cpp)
#target_link_libraries(pt_client ${catkin_LIBRARIES})
#add_dependencies(pt_client iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_impedance src/pt_impedance.cpp)
#target_link_libraries(pt_impedance ${catkin_LIBRARIES})
#add_dependencies(pt_impedance iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_joint src/pt_joint.cpp)
#target_link_libraries(pt_joint ${catkin_LIBRARIES})
#add_dependencies(pt_joint iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_admittance1 src/pt_admittance1.cpp)
#target_link_libraries(pt_admittance1 ${catkin_LIBRARIES})
#add_dependencies(pt_admittance1 iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_recircle src/pt_recircle.cpp)
#target_link_libraries(pt_recircle ${catkin_LIBRARIES})
#add_dependencies(pt_recircle iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_zq src/pt_zq.cpp)
#target_link_libraries(pt_zq ${catkin_LIBRARIES})
#add_dependencies(pt_zq iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_record_F src/pt_record_F.cpp)
#target_link_libraries(pt_record_F ${catkin_LIBRARIES})
#add_dependencies(pt_record_F iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_experiment_start src/pt_experiment_start.cpp)
#target_link_libraries(pt_experiment_start ${catkin_LIBRARIES})
#add_dependencies(pt_experiment_start iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_record_X src/pt_record_X.cpp)
#target_link_libraries(pt_record_X ${catkin_LIBRARIES})
#add_dependencies(pt_record_X iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_go src/pt_go.cpp)
#target_link_libraries(pt_go ${catkin_LIBRARIES})
#add_dependencies(pt_go iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(read src/read.cpp)
#target_link_libraries(read ${catkin_LIBRARIES})
#add_dependencies(read iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_go2 src/pt_go2.cpp)
#target_link_libraries(pt_go2 ${catkin_LIBRARIES})
#add_dependencies(pt_go2 iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(output src/example.cpp)
#target_link_libraries(output ${catkin_LIBRARIES})
#add_dependencies(output iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(read_save_X src/read_save_X.cpp)
#target_link_libraries(read_save_X ${catkin_LIBRARIES})
#add_dependencies(read_save_X iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_sj_go src/pt_sj_go.cpp)
#target_link_libraries(pt_sj_go ${catkin_LIBRARIES})
#add_dependencies(pt_sj_go iiwa_stack-master-Sunrise1.9_generate_messages_cpp)



####add qq
#add_executable(listenerRobot_Cartesianimpedancecontrol_3D_18.6.21_ATI src/qq/listenerRobot_Cartesianimpedancecontrol_3D_18.6.21_ATI.cpp)
#target_link_libraries(listenerRobot_Cartesianimpedancecontrol_3D_18.6.21_ATI ${catkin_LIBRARIES})
#add_dependencies(listenerRobot_Cartesianimpedancecontrol_3D_18.6.21_ATI iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(listenerRobot_Cartesianimpedancecontrol_3D_2.5 src/qq/listenerRobot_Cartesianimpedancecontrol_3D_2.5.cpp)
#target_link_libraries(listenerRobot_Cartesianimpedancecontrol_3D_2.5 ${catkin_LIBRARIES})
#add_dependencies(listenerRobot_Cartesianimpedancecontrol_3D_2.5 iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz src/qq/listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz.cpp)
#target_link_libraries(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz ${catkin_LIBRARIES})
#add_dependencies(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz_fast src/qq/listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz_fast.cpp)
#target_link_libraries(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz_fast ${catkin_LIBRARIES})
#add_dependencies(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz_fast iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz_fast_lvbo src/qq/listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz_fast_lvbo.cpp)
#target_link_libraries(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz_fast_lvbo ${catkin_LIBRARIES})
#add_dependencies(listenerRobot_Cartesianimpedancecontrol_3D_2.5_xyz_fast_lvbo iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_admittance src/qq/pt_admittance.cpp)
#target_link_libraries(pt_admittance ${catkin_LIBRARIES})
#add_dependencies(pt_admittance iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_admittance_add_force src/qq/pt_admittance_add_force.cpp)
#target_link_libraries(pt_admittance_add_force ${catkin_LIBRARIES})
#add_dependencies(pt_admittance_add_force iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_admittance_add_force_2d src/qq/pt_admittance_add_force_2d.cpp)
#target_link_libraries(pt_admittance_add_force_2d ${catkin_LIBRARIES})
#add_dependencies(pt_admittance_add_force_2d iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_zq1 src/qq/pt_zq1.cpp)
#target_link_libraries(pt_zq1 ${catkin_LIBRARIES})
#add_dependencies(pt_zq1 iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(pt_zq1_2d src/qq/pt_zq1_2d.cpp)
#target_link_libraries(pt_zq1_2d ${catkin_LIBRARIES})
#add_dependencies(pt_zq1_2d iiwa_stack-master-Sunrise1.9_generate_messages_cpp)






#######the commend of switch experiment

##	target_link_libraries(pt_sj_go_SEDS ${catkin_LIBRARIES})
#	add_dependencies(pt_sj_go_SEDS iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(Cartesian_velocity_to_joint_velocity src/SEDS_GMM/Cartesian_velocity_to_joint_velocity.cpp)
#target_link_libraries(Cartesian_velocity_to_joint_velocity ${catkin_LIBRARIES})
#add_dependencies(Cartesian_velocity_to_joint_velocity iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#########add_executable(readGMM src/SEDS_GMM/readGMM.cpp 
##########include/Myadd.h src/Myadd.cpp
##########~/home/edward/kuka_catkin/src/mathlib/include/Mathlib.h
##########~/home/edward/kuka_catkin/src/mathlib/src/Mathlib.cpp
##########~/home/edward/kuka_catkin/src/seds/include/seds.h
##########~/home/edward/kuka_catkin/src/seds/src/seds.cpp
######### )

#########add_library(Myadd include/Myadd.h src/Myadd.cpp)
##########add_library(mathlib 
###########libmathlib.so
###########${${PROJECT_NAME}_SRCS} 
##########include/MathLib.h src/MathLib.cpp
##########)
##########add_library(seds 
###########libseds.so
###########${${PROJECT_NAME}_SRCS} 
##########include/GMR.h src/GMR.cpp
##########)
#########target_link_libraries(readGMM 
#########				${catkin_LIBRARIES} 
#########				Myadd
#########				mathlib
#########				seds
#########		)
#########add_dependencies(readGMM iiwa_stack-master-Sunrise1.9_generate_messages_cpp) 

#add_executable(csdn_pinv src/SEDS_GMM/csdn_pinv.cpp)
#target_link_libraries(csdn_pinv ${catkin_LIBRARIES})
#add_dependencies(csdn_pinv iiwa_stack-master-Sunrise1.9_generate_messages_cpp)

#add_executable(svd_pinv src/SEDS_GMM/svd_pinv.cpp)
#target_link_libraries(svd_pinv ${catkin_LIBRARIES})
#add_dependencies(svd_pinv iiwa_stack-master-Sunrise1.9_generate_messages_cpp)


#add_executable(Myadd include/Myadd.h src/Myadd.cpp)
#target_link_libraries(Myadd ${catkin_LIBRARIES})
#add_dependencies(Myadd iiwa_stack-master-Sunrise1.9_generate_messages_cpp)









