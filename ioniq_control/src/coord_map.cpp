                       //
//  coord_map.cpp
//  MAIN
//
//  Created by Changhyeon Park on 04/09/2018.
//
//#include "option.h"


//#define OpenCV_View_MAP
//#ifdef OpenCV_View_MAP
//#include <opencv2/opencv.hpp>
//#endif

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include "coord_map.h"
#include "env_setting.h"

// latitude, longitude, heading;
// x = 0;
// y = 0;
// WP_address = 0;
// WP_next_address = 0;
// WP_target_speed = 0;
// WP_stop_line_flag = 0;
// Make a variable for saving Coordinates Map Data
//coordinates_map *coord_map;

void Coordinates_Map_Data()
{
    coord_map = (coordinates_map*)malloc(50000 * sizeof(coordinates_map));
    
#ifdef OpenCV_View_MAP
    //K-city
    cv::Mat img_coord_map(700, 1800, CV_8UC1);

    // Ochang
    //cv::Mat img_coord_map(700, 300, CV_8UC1);
    img_coord_map.setTo(0);
#endif
    
    std::ifstream fin;
    
#ifdef __APPLE__
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/GNSS/Coordinates_Map/gnss_receive_data_좌회전_10kph_modified.txt");
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/gnss_receive_data__gg.txt");
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/gnss_receive_data_tmp3.txt");
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/GNSS/gnss_receive_data_rtk_좌회전_2.txt");
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/GNSS/Coordinates_Map/gnss_receive_data_직진_10kph.txt");

//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/GNSS/Coordinates_Map/gnss_receive_data_9_18.txt");
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/GNSS/Coordinates_Map/gnss_receive_data_9_18_2.txt");
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/GNSS/Coordinates_Map/gnss_receive_data_9_18_3.txt");
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/GNSS/Coordinates_Map/gnss_receive_data_vehicle_speed_5hz.txt");
    fin.open("../../GNSS/Coordinates_Map/gnss_receive_data_vehicle_speed_5hz.txt");
#else
    //fin.open("/home/chp/ioniq/vehicle_control/Code_CHP/PEAKCAN_Mac/c++/GNSS/Coordinates_Map/gnss_receive_data_9_18_2.txt");
//    fin.open("/home/chp/ioniq/vehicle_control/Code_CHP/PEAKCAN_Mac/c++/GNSS/Coordinates_Map/gnss_receive_data_cbnu_rtk.txt");
//    fin.open("../c++/GNSS/Coordinates_Map/gnss_receive_data_cbnu_rtk.txt");

    //in ros,,
    //fin.open("../../../src/darknet_ros/ioniq_control/GNSS/Coordinates_Map/gnss_receive_data_vehicle_speed_5hz.txt");
//    fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/GNSS/Coordinates_Map/gnss_receive_data_vehicle_speed_5hz.txt");
//    fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/GNSS/Coordinates_Map/gnss_receive_data_직진.txt");
//    fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/GNSS/Coordinates_Map/gnss_receive_data_시나리오1.txt");
    fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/GNSS/Coordinates_Map/gnss_receive_data_시나리오3.txt");


    //fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/GNSS/Coordinates_Map/gnss_receive_data_k_city_2.txt");

    //fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/GNSS/Coordinates_Map/gnss_receive_data_longitudinal_test.txt");

    //fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/GNSS/Coordinates_Map/gnss_receive_data_rtk_slow.txt");


#endif
    
    
    int i = 0;

//    while(fin>>coord_map[i].latitude>>coord_map[i].longitude>>coord_map[i].heading>>coord_map[i].WP_address>>coord_map[i].WP_next_address>>coord_map[i].WP_target_speed>>coord_map[i].WP_stop_line_flag)

    while(fin>>coord_map[i].latitude>>
          coord_map[i].longitude>>
          coord_map[i].heading>>
          coord_map[i].WP_target_speed>>
          coord_map[i].WP_stop_line_address>>
          coord_map[i].WP_stop_line_latitude>>
          coord_map[i].WP_stop_line_longitude)
    {
        // Starting point   : 36.72790320        127.44262740
        // longitude        : 111km per 1degree.                    = 1.1cm per 0.0000001degree
        // latitude         : 88.8km per 1degree at longitude 37    = 0.88cm per 0.0000001degree
        
        
        // How to change Wold Coordinate to Relative Coordinate
        // latitude(cm) = (latitude(degree) - Starting point(degree)) * 10,000,000 * 0.888
        // longitude(cm) = (longitude(degree) - Starting point(degree)) * 10,000,000 * 1.1
        
//        double last_x = coord_map[i].x;
//        double last_y = coord_map[i].y;
        
        // k-city offset ;; x = 300, y = 120
        // ochang offset : x = 30, y = 10;
        // calculated as m (+ 30)
        coord_map[i].x =(coord_map[i].latitude - base_latitude)* 10000000 * 0.888 / 100. + 30;
        coord_map[i].y =(coord_map[i].longitude - base_longitude) * 10000000 * 1.1 / 100. + 10;
        
//        double distance = sqrt((coord_map[i].x-last_x)*(coord_map[i].x-last_x) + (coord_map[i].y-last_y) * (coord_map[i].y-last_y));
        
        std::cout << std::fixed;
        std::cout.precision(7);
        std::cout << coord_map[i].x <<"\t\t"<< coord_map[i].y << "\t\t" << coord_map[i].heading <<"\t\t" << "\t\t" << coord_map[i].WP_target_speed <<"\t\t"<< std::endl;

#ifdef OpenCV_View_MAP
        //        img_coord_map.data[(int)(coord_map[i].y*4) * img_coord_map.cols + (int)(coord_map[i].x*4)] = 0;
        
        // viewer에 그리기
        cv::Point dot(coord_map[i].x*4, coord_map[i].y*4);
        cv::circle(img_coord_map, dot, 1, CvScalar(255,0,255));
        
        // 좌표지도용 좌표이미지에 그리기
        //cv::imshow("Coordinate_map", img_coord_map);
        
        // 실시간 좌표이미지에 그리기
        cv::circle(img, dot, 3, CvScalar(0,255,0));
        
//        cv::imshow("Coordinate_map", img);
        cv::waitKey(1);
#endif
        i++;
    }
    
    int afdas = 3;
    fin.close();
}
