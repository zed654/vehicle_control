cmake_minimum_required(VERSION 2.6)

project(pcan_mac)

find_package(serial)
find_package(OpenCV)

include_directories(
	/usr/local/include
	/usr/include
	include
    include/serial
    include/serial/impl
	src
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#   include_directories(/Users/macbookpro/ioniq/Code_CHP/PEAKCAN_Mac/c++)

#   link_directories(/usr/local/lib)
#   link_libraries(libPCBUSB.dylib)

link_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/lib
	/tmp/usr/local/lib
    /usr/local/lib/lib
    /usr/local/lib
)
link_libraries(
	libPCBUSB.dylib
	libserial.dylib
) # libPCBUSB.0.8.dylib

#   include_directories(/Users/macbookpro/Downloads/PCBUSB/examples/c++/include)
#   link_directories(/Users/macbookpro/Downloads/PCBUSB/examples/c++/lib)
#   link_libraries(/Users/macbookpro/Downloads/PCBUSB/examples/c++/lib/libPCBUSB.0.8.dylib)

# Add OpenCV Libraries
link_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})


#add_executable(pcan_rx src/pcbusb_recv.cpp)
#add_executable(pcan_tx src/pcbusb_send.cpp include/can_mac_val.h)
#add_executable(receive_steer_JY src/receive_steer_JY.cpp)
#add_executable(ASM_controlg src/control_ASM.cpp)
#add_executable(TEST src/pcbusb_send_test.cpp)

add_executable(MAIN
    src/main.cpp
    src/parser_vercpp.cpp
    src/coord_map.cpp
    src/CAN_RW__thread__.cpp
    src/GNSS_receive__thread__.cpp
    src/dead_reckoning__thread__.cpp
    src/path_follow__thread__.cpp

    include/env_setting.h
    include/pd_control.h
    include/coord_map.h
    include/parser_vercpp.hpp
)

#add_executable(MAP_VIEWER src/map_viewer.cpp)
#add_executable(PEAKCAN src/control_peakcan.cpp)
#add_executable(new_peacan src/new_peak_can_test.cpp)
add_executable(C_TEST src/c_test.cpp src/c_test_fun.cpp include/c_test.h)
#add_executable(COORDINATES_MAP_READER src/coordinates_map_reader.cpp)

# target_link_libraries have to located after add_executable
#target_link_libraries(C_TEST ${OpenCV_LIBS})
