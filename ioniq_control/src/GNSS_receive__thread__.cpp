
/*
    GNSS(C94-M8P)를 이용한 좌표 receive 스레드
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include "env_setting.h"
#include "parser_vercpp.hpp"
#include "coord_map.h"

// Read Coordinates Map Data
//void Coordinates_Map_Data();

//#define RECEIVE_FREQ(hz) 5/hz
//
//#ifdef RECEIVE_FREQ(5)
//#endif

//#define OLD_VIEWER
#define NEW_VIEWER

#define RECEIVE_FREQ(hz) 5/hz
#define RECEIVE_FREQ_VALUE 5

void* GNSS_Receive(void *gnss_error_flag_)
{

#if defined (GNSS_PORT_PATH) && (GNSS_BAUD_RATE)
#else
    printf("GNSS device env setting is not defined by #define comment\n\n");
#endif // defined (GNSS_PORT_PATH) && (GNSS_BAUD_RATE)
    
#ifdef GNSS_DATA_WRITE
    std::ofstream fin;

#ifdef __APPLE__
    fin.open("../../GNSS/gnss_receive_data.txt");
#else
    //fin.open("../c++/gnss_receive_data.txt");
    fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/GNSS/gnss_receive_data.txt");
#endif  //__APPLE__
    
#endif // GNSS_DATA_WRITE
    
    //    int baudrate = atoi(baud_str.c_str());
    
    ublox_parser cUblox(GNSS_PORT_PATH, GNSS_BAUD_RATE);
    ublox_parser::PARSING_TYPEDEF_UBX_M8P_PVT globalPVT;
    
#ifdef RECEIVE_FREQ
    int GNSS_Hz_count = 0;
#endif // RECEIVE_FREQ
    
    while (cUblox.isInit)
    {
        gnss_initial_flag = 1;
        
        //        // check running time
        //        struct timeval t1, t2;
        //        double elapsedTime;
        //
        //        // start timer
        //        gettimeofday(&t1, NULL);
        
        cUblox.run();
        
        if (cUblox.valid() == ublox_parser::PARSING_SUCCESS_)
        {
#ifdef RECEIVE_FREQ
            GNSS_Hz_count++;
            
            if(GNSS_Hz_count == RECEIVE_FREQ(RECEIVE_FREQ_VALUE)) // RECEIVE_FREQ(Hz)
            {
            cUblox.copyTo(&globalPVT);
//            std::cout << std::fixed;
//            std::cout.precision(8);
//            std::cout<< globalPVT.lat << ", " << globalPVT.lon << ", " << globalPVT.headMot << std::endl;
            
#ifdef GNSS_DATA_WRITE
            // Starting point   : 36.72790320        127.44262740
            // longitude        : 111km per 1degree.                    = 1.1cm per 0.0000001degree
            // latitude         : 88.8km per 1degree at longitude 37    = 0.88cm per 0.0000001degree
            
            // How to change Wold Coordinate to Relative Coordinate
            // latitude(cm) = (latitude(degree) - Starting point(degree)) * 10,000,000 * 0.888
            // longitude(cm) = (longitude(degree) - Starting point(degree)) * 10,000,000 * 1.1
            
            fin << std::fixed;
            fin.precision(8);
            fin << globalPVT.lat << "\t\t";
            fin << globalPVT.lon << "\t\t";
            fin << globalPVT.headMot << "\t\t";
            fin << current_vehicle_speed << "\t\t";
            fin << 0 << "\t\t";
            fin << 0 << "\t\t";
            fin << 0 << std::endl;
#endif // GNSS_DATA_WRITE
            
#ifdef OLD_VIEWER

            // calculated as m. and It was shifted x axis 30m, y axis 10m
            // 충북대 본캠 : 36.62489480, 127.45667630
            // 충북대 오창 : 36.72790320        127.44262740
            //            gnss_x =(globalPVT.lat - 36.62489480)* 10000000 * 0.888 / 100. + 30;
            //            gnss_y =(globalPVT.lon - 127.45667630) * 10000000 * 1.1 / 100. + 10;
            
            // 충북대 본캠에서 테스트하기 위한 코드. 오창PG에서는 _tmp를 우측 주석부분껄 넣어야함.
            double base_latitude_tmp = base_latitude; //36.62489480;
            double base_longitude_tmp = base_longitude;//127.45667630;   //base_longitude;
            // k-city offset ;; x = 300, y = 120
            // ochang offset : x = 30, y = 10
        

//                gnss_x =(globalPVT.lat - base_latitude_tmp)* 10000000 * 0.888 / 100. + 30;//100; // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
//                gnss_y =(globalPVT.lon - base_longitude_tmp) * 10000000 * 1.1 / 100. + 10;//30;  // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
                
                gnss_x =(globalPVT.lat - base_latitude_tmp)* 10000000 * 1.1 / 100. + 30;//100; // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
                gnss_y =(globalPVT.lon - base_longitude_tmp) * 10000000 * 0.888 / 100. + 10;//30;  // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
                
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
#endif  // OpenCV_View_MAP
//            }
#endif // OLD_VIEWER




#ifdef NEW_VIEWER
            
                //    left_top_x =  0;
                //    left_top_y =  0;
                //    right_bottom_x = 0;
                //    right_bottom_y = 0;
            
                //    opencv_viewer_zoom = 4;         // 맵이 너무 크면, opencv_viewer_zoom을 줄이면 됨.
                //    opencv_view_point_margin = 50;
                //    opencv_viewer_width = 0;
                //    opencv_viewer_height = 0;

                double gnss_x_tmp = 0;
                double gnss_y_tmp = 0;

                
                //111.111*cos(37*3.141592/180) = 88.73
            

                
                
                gnss_x_tmp =(globalPVT.lat - base_latitude)* 10000000 * 1.1 / 100.;     // 위도에 의한 계산 (남북)
//                gnss_y_tmp =(globalPVT.lon - base_longitude) * 10000000 * 0.888 / 100.; // 경도에 의한 계산 (동서)
                gnss_y_tmp =(globalPVT.lon - base_longitude)* 1000. * 111.111 * cos(base_latitude * 3.141592/180);  // + 30;//100; // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
                
//                gnss_x_tmp =(globalPVT.lat - base_latitude)* 10000000 * 0.888 / 100.;// + 30;//100; // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
//                gnss_y_tmp =(globalPVT.lon - base_longitude) * 10000000 * 1.1 / 100.;// + 10;//30;  // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
                gnss_x = (gnss_x_tmp - left_top_x + opencv_view_point_margin/2);  // left_top_x가 제대로 나왔으면 coord_map[i].x - left_top_x 는 항상 양수
                gnss_y = (gnss_y_tmp - left_top_y + opencv_view_point_margin/2);
                gnss_yaw_angle = globalPVT.headMot;
                GNSS_heading = globalPVT.headMot;
                GNSS_Hz_count = 0;
            
#ifdef OpenCV_View_MAP
                // viewer에 그리기
                cv::Point dot(gnss_x * opencv_viewer_zoom, gnss_y * opencv_viewer_zoom);
                cv::circle(img, dot, 6, CvScalar(255,255,255));
#endif // OpenCV_View_MAP
            
#endif // NEW_VIEWER
            
#endif // RECEIVE_FREQ
            }
//            gnss_x =(globalPVT.lat - base_latitude_tmp)* 10000000 * 0.888 / 100. + 30;//100; // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
//            gnss_y =(globalPVT.lon - base_longitude_tmp) * 10000000 * 1.1 / 100. + 10;//30;  // Visualize를 위해 (0, 0)으로부터 좌표이동도 추가
//            gnss_yaw_angle = globalPVT.headMot;
//            GNSS_heading = globalPVT.headMot;

            //std::cout << gnss_x << "\t\t" << gnss_y << std::endl;
            
            //            std::cout << std::fixed;
            //            std::cout.precision(2);
            //            std::cout<< gnss_x << "\t\t" << gnss_y << "\t\t" << gnss_yaw_angle << "\t\t" << "GNSS" << std::endl;
            


//            if(vehicle_yaw_rate_error_correct == 1)
//                std::cout << "target_vehicle_speed = " << target_vehicle_speed << "\t\tvehicle_speed =" << current_vehicle_speed << std::endl;
        }



        usleep(1);
    }
    
#ifdef GNSS_DATA_WRITE
    fin.close();
#endif
    return 0;   // 원래는 해줘야함. return (void *)value;
}
