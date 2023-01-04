cmake_minimum_required(VERSION 3.0.2)
project(sr80_hardware_interface)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
)
find_library(/home/$ENV{USER}/ADS/build/libAdsLib.a REQUIRED)

set(AdsLib_INCLUDE_DIRS /home/$ENV{USER}/ADS/AdsLib/) 

catkin_package(
    INCLUDE_DIRS include ${AdsLib_INCLUDE_DIRS}
    LIBRARIES sr80_hardware_interface
    CATKIN_DEPENDS 
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
    DEPENDS /home/$ENV{USER}/ADS/build/libAdsLib.a
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${AdsLib_INCLUDE_DIRS}
)

add_executable(sr80_hwi_node 
    src/sr80_hwi_node.cpp
    src/sr80_hw_interface.cpp
    src/sr80_hw.cpp
    src/sr80_interface.cpp
    src/ads_worker.cpp
)

target_link_libraries(sr80_hwi_node ${catkin_LIBRARIES} /home/$ENV{USER}/ADS/build/libAdsLib.a)
target_link_directories(sr80_hwi_node PUBLIC ${AdsLib_INCLUDE_DIRS})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

