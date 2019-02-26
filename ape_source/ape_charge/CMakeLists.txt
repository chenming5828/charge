cmake_minimum_required(VERSION 2.8.3)
project(ape_charge)

set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )


find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf2
            tf2_msgs
            tf2_ros
            tf2_geometry_msgs
            nav_msgs
            actionlib_msgs
            actionlib
            message_filters
            sensor_msgs
            ape_msgs
            ape_actions
 
        )
         

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})



find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})


find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)


include_directories(include  ${csm_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})



catkin_package(
)


add_executable(odom2map src/odom2map.cpp  )
target_link_libraries(odom2map
    ${csm_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

# add_executable(showpose src/showpose.cpp  )
# target_link_libraries(showpose
#     ${Boost_LIBRARIES}
#     ${catkin_LIBRARIES}
# )

# add_executable(showodom src/showodom.cpp  )
# target_link_libraries(showodom
#     ${Boost_LIBRARIES}
#     ${catkin_LIBRARIES}
# )




add_executable(showlaser src/showlaser.cpp  )
target_link_libraries(showlaser
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)




add_executable(charge_navigation_client src/charge_navigation_client.cpp  )
target_link_libraries(charge_navigation_client
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(charge_navigation_server src/charge_navigation_server.cpp  )
target_link_libraries(charge_navigation_server
    ${csm_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)












