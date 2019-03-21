cmake_minimum_required(VERSION 2.8.3)

project(localization)


list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  geometry_msgs
  eigen_conversions
)

find_PACKAGE(cmake_modules REQUIRED)

FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(Eigen3 REQUIRED)

add_definitions(-DEIGEN_DONT_VECTORIZE)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

FIND_PACKAGE(tf REQUIRED)

find_library(GSL_LIBRARY
        NAMES gsl libgsl
        PATHS  ${GSL_POSSIBLE_ROOT_DIRS}
        PATH_SUFFIXES lib
        DOC "GSL library"
)

FIND_LIBRARY(GSLCBLAS_LIBRARY
        NAMES gslcblas libgslcblas
        PATHS  ${GSL_POSSIBLE_ROOT_DIRS}
        PATH_SUFFIXES lib
        DOC "GSL cblas library"
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   OutputData.msg
   Particle.msg
   ParticleSet.msg
   GoalpostsDetected.msg
   WorldObjects.msg
   LinesDetected.msg
   ObjectsDetected.msg
   MeanPoseConfStamped.msg
   ObstaclesDetected.msg
 )

generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
   tf
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES localization eigen gsl
  CATKIN_DEPENDS message_runtime cv_bridge message_generation dynamic_reconfigure image_transport roscpp rospy sensor_msgs std_msgs geometry_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${Eigen_INCLUDE_DIRS})

add_executable(loc_head_odom_broadcaster src/loc_head_odom_broadcaster.cpp)
target_link_libraries(loc_head_odom_broadcaster ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(loc_head_odom_broadcaster localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(test_odom src/test_odom.cpp)
#target_link_libraries(test_odom ${catkin_LIBRARIES})
#add_dependencies(test_odom localization_gencpp)

add_executable(behavior_initial_pose src/behavior_initial_pose.cpp)
target_link_libraries(behavior_initial_pose ${catkin_LIBRARIES})
add_dependencies(behavior_initial_pose behavior_gencpp)
target_link_libraries(behavior_initial_pose ${GSL_LIBRARY} ${GSLCBLAS_LIBRARY})
target_link_libraries(behavior_initial_pose field_model) #link this executable with the field_model library
add_dependencies(behavior_initial_pose localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(behavior_head src/behavior_head.cpp)
target_link_libraries(behavior_head ${catkin_LIBRARIES})
add_dependencies(behavior_head localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(data_input_output src/data_input_output.cpp)
target_link_libraries(data_input_output ${catkin_LIBRARIES})
add_dependencies(data_input_output localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(data_input_output_sim src/data_input_output_sim.cpp)
target_link_libraries(data_input_output_sim ${catkin_LIBRARIES})
add_dependencies(data_input_output_sim localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(field_model src/field_model.cpp) # adding the field_model as a library
target_link_libraries(field_model ${catkin_LIBRARIES})
add_dependencies(field_model localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(particle_filter_localization src/particle_filter_localization.cpp)
target_link_libraries(particle_filter_localization ${catkin_LIBRARIES})
target_link_libraries(particle_filter_localization ${GSL_LIBRARY} ${GSLCBLAS_LIBRARY})
target_link_libraries(particle_filter_localization field_model) #link this executable with the field_model library
add_dependencies(particle_filter_localization localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(particle_filter_localization_sim src/particle_filter_localization_sim.cpp)
#target_link_libraries(particle_filter_localization_sim ${catkin_LIBRARIES})
#target_link_libraries(particle_filter_localization_sim ${GSL_LIBRARY} ${GSLCBLAS_LIBRARY})
#target_link_libraries(particle_filter_localization_sim field_model) #link this executable with the field_model library
#add_dependencies(particle_filter_localization_sim localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(localization_visualization src/localization_visualization.cpp)
target_link_libraries(localization_visualization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(localization_visualization field_model) #link this executable with the field_model library
add_dependencies(localization_visualization localization_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})



