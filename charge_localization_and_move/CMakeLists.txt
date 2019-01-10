cmake_minimum_required(VERSION 2.8.3)
project(charge_localization_and_move)

set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )


find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            std_srvs
            tf
            nav_msgs
            std_srvs
            message_generation
        )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})



find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)


include_directories(include  ${csm_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

add_message_files(
  FILES
  posewithscan.msg
)
 
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
 


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES icp_precise
    CATKIN_DEPENDS message_runtime
)


add_executable(odom2map src/odom2map.cpp  )
target_link_libraries(odom2map
    ${csm_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(showpose src/showpose.cpp  )
target_link_libraries(showpose
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(showlaser src/showlaser.cpp  )
target_link_libraries(showlaser
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)


# ADD_LIBRARY(icp_precise src/icp_pose.cpp)

# add_executable(charge_node src/charge_nav.cpp  )
# target_link_libraries(charge_node
#     icp_precise
#     ${csm_LIBRARIES}
#     ${Boost_LIBRARIES}
#     ${catkin_LIBRARIES}
# )
















