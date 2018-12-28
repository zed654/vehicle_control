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

//#define OLD_VIEWER
#define NEW_VIEWER

double return_left_top_x(coordinates_map *coord_map_, int coord_map_size_);
double return_left_top_y(coordinates_map *coord_map_, int coord_map_size_);
double return_right_bottom_x(coordinates_map *coord_map_, int coord_map_size_);
double return_right_bottom_y(coordinates_map *coord_map_, int coord_map_size_);
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
    
//#ifdef OpenCV_View_MAP
//    //K-city
//    cv::Mat img_coord_map(700, 1800, CV_8UC1);
//
//    // Ochang
//    //cv::Mat img_coord_map(700, 300, CV_8UC1);
//    img_coord_map.setTo(0);
//#endif
    
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
//    fin.open("../../GNSS/Coordinates_Map/gnss_receive_data_vehicle_speed_5hz.txt");
    
//    fin.open("../../GNSS/gnss_receive_data20181123_1.txt");
//    fin.open("../../GNSS/gnss_receive_data_1205.txt");
//    fin.open("../../GNSS/gnss_receive_data_1205_2.txt");
//    fin.open("../../GNSS/gnss_receive_data_1205_3.txt");
//    fin.open("../../GNSS/gnss_receive_data_1205_55.txt");
//    fin.open("../../GNSS/gnss_receive_data_error_2.txt");
//    fin.open("../../GNSS/gnss_receive_data_PP.txt");
    
//    fin.open("../../GNSS/Coordinates_Map/gnss_receive_data_k_city_2.txt");
//    fin.open("../../GNSS/gnss_receive_data_NEW_VIEWER_2.txt");
    
//    fin.open("../../GNSS/gnss_receive_data_k-city_straight.txt");

//    fin.open("../../GNSS/gnss_receive_data_school.txt");

//    fin.open("../../GNSS/gnss_receive_data_20181213_school.txt");
    
    fin.open("../../GNSS/gnss_receive_data_20181214.txt");
    
    
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

#ifdef OLD_VIEWER
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

        coord_map[i].x =(coord_map[i].latitude - base_latitude)* 10000000 * 1.1 / 100. + 30;
        coord_map[i].y =(coord_map[i].longitude - base_longitude) * 10000000 * 0.888 / 100. + 10;
        
//        coord_map[i].x =(coord_map[i].latitude - base_latitude)* 10000000 * 0.888 / 100. + 30;
//        coord_map[i].y =(coord_map[i].longitude - base_longitude) * 10000000 * 1.1 / 100. + 10;
        
//        double distance = sqrt((coord_map[i].x-last_x)*(coord_map[i].x-last_x) + (coord_map[i].y-last_y) * (coord_map[i].y-last_y));
        
        std::cout << std::fixed;
        std::cout.precision(7);
        std::cout << coord_map[i].x <<"\t\t"<< coord_map[i].y << "\t\t" << coord_map[i].heading <<"\t\t" << "\t\t" << coord_map[i].WP_target_speed <<"\t\t"<< std::endl;

#ifdef OpenCV_View_MAP
        //        img_coord_map.data[(int)(coord_map[i].y*4) * img_coord_map.cols + (int)(coord_map[i].x*4)] = 0;
        
        // viewer에 그리기
        cv::Point dot(coord_map[i].x*4, coord_map[i].y*4);
//        cv::circle(img_coord_map, dot, 1, CvScalar(255,0,255));
        
        // 좌표지도용 좌표이미지에 그리기
        //cv::imshow("Coordinate_map", img_coord_map);
        
        // 실시간 좌표이미지에 그리기
        cv::circle(img, dot, 3, CvScalar(0,255,0));
        
//        cv::imshow("Coordinate_map", img);
        cv::waitKey(1);

#endif
        i++;
    }
#endif

#ifdef NEW_VIEWER
    
    coordinates_map *coord_map_tmp;
    coord_map_tmp = (coordinates_map*)malloc(50000 * sizeof(coordinates_map));

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
        

        
        coord_map_tmp[i].x =(coord_map[i].latitude - base_latitude)* 10000000 * 1.1 / 100.;
//        coord_map_tmp[i].y =(coord_map[i].longitude - base_longitude) * 10000000 * 0.888 / 100.;
        coord_map_tmp[i].y =(coord_map[i].longitude - base_longitude)* 1000. * 111.111 * cos(base_latitude * 3.141592/180);  // + 30;//100; // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
        
//        coord_map_tmp[i].x =(coord_map[i].latitude - base_latitude)* 10000000 * 0.888 / 100.;
//        coord_map_tmp[i].y =(coord_map[i].longitude - base_longitude) * 10000000 * 1.1 / 100.;
        i++;
    }
    
    
    
//    left_top_x =  0;
//    left_top_y =  0;
//    right_bottom_x = 0;
//    right_bottom_y = 0;
    
//    opencv_viewer_zoom = 4;         // 맵이 너무 크면, opencv_viewer_zoom을 줄이면 됨.
//    opencv_view_point_margin = 50;
//    opencv_viewer_width = 0;
//    opencv_viewer_height = 0;
    
    coord_map_size = i;
    
    left_top_x = return_left_top_x(coord_map_tmp, coord_map_size);
    left_top_y = return_left_top_y(coord_map_tmp, coord_map_size);
    right_bottom_x = return_right_bottom_x(coord_map_tmp, coord_map_size);
    right_bottom_y = return_right_bottom_y(coord_map_tmp, coord_map_size);
    
    opencv_viewer_width = (abs(left_top_x - right_bottom_x) + opencv_view_point_margin) * opencv_viewer_zoom;
    opencv_viewer_height = (abs(left_top_y - right_bottom_y) + opencv_view_point_margin) * opencv_viewer_zoom;
    
    cv::resize(img, img, cv::Size(opencv_viewer_width, opencv_viewer_height), 0, 0, CV_INTER_LINEAR);

    for(int i = 0; i < coord_map_size; i++)
    {
        coord_map[i].x = (coord_map_tmp[i].x - left_top_x + opencv_view_point_margin/2);  // left_top_x가 제대로 나왔으면 coord_map[i].x - left_top_x 는 항상 양수
        coord_map[i].y = (coord_map_tmp[i].y - left_top_y + opencv_view_point_margin/2);
        
#ifdef OpenCV_View_MAP
        
        // viewer에 그리기
        cv::Point dot(coord_map[i].x * opencv_viewer_zoom, coord_map[i].y * opencv_viewer_zoom);
//        cv::circle(img_coord_map, dot, 1, CvScalar(255,0,255));
        
        // 실시간 좌표이미지에 그리기
        cv::circle(img, dot, 3, CvScalar(0,255,0));
        
        cv::waitKey(1);
        
#endif
        
    }
    
    int basidcabsuicas=3;
    
#endif
    /*
     ////////////////////////////////////////////////////////////////////////
     0. coord_map의 x, y좌표 구하기
     ////////////////////////////////////////////////////////////////////////
     while(fin << xxx)
     {
        // base_latitude, base_longitude는 오창 성능시험장 입구
        coord_map[i].x =(coord_map[i].latitude - base_latitude)* 10000000 * 0.888 / 100.;
        coord_map[i].y =(coord_map[i].longitude - base_longitude) * 10000000 * 1.1 / 100.;
        i++;
     }
     
     
     
     
     ////////////////////////////////////////////////////////////////////////
     1. 좌측상단좌표, 우측하단좌표 구하기 (left_top_x, left_top_y), (right_bottom_x, right_bottom_y)
     ////////////////////////////////////////////////////////////////////////
     
     // i에 카운트값 있음.
     int coord_map_size = i;
     
     left_top_x =  return_left_top_x(coord_map, coord_map_size);
     left_top_y =  return_left_top_y(coord_map, coord_map_size);
     right_bottom_x =  return_right_bottom_x(coord_map, coord_map_size);
     right_bottom_y =  return_right_bottom_y(coord_map, coord_map_size);
     
     
     
     ////////////////////////////////////////////////////////////////////////
     2. 1. 을 기반으로 OpenCV Viewer Resize
     ////////////////////////////////////////////////////////////////////////
     double opencv_viewer_zoom = 4;         // 맵이 너무 크면, opencv_viewer_zoom을 줄이면 됨.
     double opencv_view_point_margin = 50;
     double opencv_viewer_width = 0;
     double opencv_viewer_height = 0;
     
     opencv_viewer_width = (abs(left_top_x - right_bottom_x) + opencv_view_point_margin) * opencv_viewer_zoom;
     opencv_viewer_height = (abs(left_top_y - right_bottom_y) + opencv_view_point_margin) * opencv_viewer_zoom;
     
     cv::resize(img, img, cv::Size(opencv_viewer_width, opencv_viewer_height), 0, 0, CV_INTER_LINEAR);

     for(int i = 0; i < coord_map의 개수; i++)
     {
        coord_map[i].x = (coord_map[i].x - left_top_x + opencv_view_point_margin/2) * opencv_viewer_zoom;  // left_top_x가 제대로 나왔으면 coord_map[i].x - left_top_x 는 항상 양수
        coord_map[i].y = (coord_map[i].y - left_top_y + opencv_view_point_margin/2) * opencv_viewer_zoom;
     
        ////////////////////////////////////////////////////////////////////////
        3. viewer에 그리기
        ////////////////////////////////////////////////////////////////////////
     
     #ifdef OpenCV_View_MAP
     
        // viewer에 그리기
        cv::Point dot(coord_map[i].x, coord_map[i].y);
        cv::circle(img_coord_map, dot, 1, CvScalar(255,0,255));
     
        // 실시간 좌표이미지에 그리기
        cv::circle(img, dot, 3, CvScalar(0,255,0));
     
        cv::waitKey(1);
     
     #endif
     
     }
     

     
     
     
     
     
     
     
     ////////////////////////////////////
     3. viewer에 그리기
     ////////////////////////////////////
     
     #ifdef OpenCV_View_MAP

     // viewer에 그리기
     cv::Point dot(coord_map[i].x, coord_map[i].y);
     cv::circle(img_coord_map, dot, 1, CvScalar(255,0,255));
     
     // 실시간 좌표이미지에 그리기
     cv::circle(img, dot, 3, CvScalar(0,255,0));
     
     cv::waitKey(1);
     
     #endif
    */
    
    
    
    
    
    
    /*
     ////////////////////////////////////////////////////////////////////////////////////////////////////////////
     GNSS_receive__thread__.cpp 꺼
     ////////////////////////////////////////////////////////////////////////////////////////////////////////////
     
     gnss_x =(globalPVT.lat - base_latitude_tmp)* 10000000 * 0.888 / 100. + 30;//100; // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
     gnss_y =(globalPVT.lon - base_longitude_tmp) * 10000000 * 1.1 / 100. + 10;//30;  // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
     gnss_yaw_angle = globalPVT.headMot;
     GNSS_heading = globalPVT.headMot;
     GNSS_Hz_count = 0;
     //std::cout << GNSS_Hz_count << std::endl;
     
     #ifdef OpenCV_View_MAP
     // viewer에 그리기
     cv::Point dot(gnss_x*4, gnss_y*4);
     cv::circle(img, dot, 6, CvScalar(255,255,255));
     
     //            count_gnss++;
     //            printf("count_gnss = %d\n", count_gnss);
     
     //            img.data[(int)(gnss_y*4) * img.cols + (int)(gnss_x*4)] = 255;
     // imshow는 main()에서. thread에서는 imshow 애러뜸.
     #endif
     
     
     */
    int afdas = 3;
    fin.close();
}

/*
 수정해야할 점 :
 현재 while(fin<< xxx) 를 통해 불러오면 바로 그리는 형식임. 바꿔야함.
 while(fin << xxx )를 통해 coord_map의 값을 모두 채운 후, 가장자리 픽셀을 구한다. 이를 이용해 OpenCV Viewer의 크기를 설정하고, 이에 따라 coord_map의 좌표계를 변환하여 OpenCV Viewer에 맞춰서 그린다. (마진은 50픽셀씩이면 좋을 듯)
 
 유의점 :
 그리는 값 = 포지션 좌표 * 4
 */


double return_left_top_x(coordinates_map *coord_map_, int coord_map_size_)
{
    double tmp_ = coord_map_[0].x;
    for(int i = 1; i < coord_map_size_; i++)
    {
        // 작은거 찾기
        tmp_ = coord_map_[i].x > tmp_ ? tmp_ : coord_map_[i].x;
    }
    return tmp_;
}

double return_left_top_y(coordinates_map *coord_map_, int coord_map_size_)
{
    double tmp_ = coord_map_[0].y;
    for(int i = 1; i < coord_map_size_; i++)
    {
        // 작은거 찾기
        tmp_ = coord_map_[i].y > tmp_ ? tmp_ : coord_map_[i].y;
    }
    return tmp_;
}

double return_right_bottom_x(coordinates_map *coord_map_, int coord_map_size_)
{
    double tmp_ = coord_map_[0].x;
    for(int i = 1; i < coord_map_size_; i++)
    {
        // 작은거 찾기
        tmp_ = coord_map_[i].x < tmp_ ? tmp_ : coord_map_[i].x;
    }
    return tmp_;
}

double return_right_bottom_y(coordinates_map *coord_map_, int coord_map_size_)
{
    double tmp_ = coord_map_[0].y;
    for(int i = 1; i < coord_map_size_; i++)
    {
        // 작은거 찾기
        tmp_ = coord_map_[i].y < tmp_ ? tmp_ : coord_map_[i].y;
    }
    return tmp_;
}
