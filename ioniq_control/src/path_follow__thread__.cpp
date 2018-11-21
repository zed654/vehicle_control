//
//  path_follow.cpp
//  MAIN
//
//  Created by Changhyeon Park on 07/09/2018.
//
#include "pd_control.h"
#include <stdio.h>
#include <math.h>
#include "env_setting.h"
#include "coord_map.h"
#include <sys/time.h>
#include <unistd.h>
//#include "TSR_receive.h"
#include <queue>

//#define LATERAL_CONTROL_PRINT
#define STEERING_ANGLE_MOVING_AVG

// coord_map[1].x
// coord_map[1].y

// dr_x
// dr_y
void waypoint_change();
void longitude_control();
void stanley_steering_control();
double calc_two_point_distance(double x_1_, double y_1_, double x_2_, double y_2_);
int rover_steering_angle_direction();  // 차량(Rover)가 진행 방향의 좌측인지 우측인지 확인하는 함수

// 좌표지도는 0부터 시작. 0~1의 Path를 표현하기 위해 첫 좌표지도 포인트는 1부터 시작한다.
//static int coord_current_address = 1;

static double Rover2WP_distance = 0;


// stanley 필요한 변수
//static double psi = 0;                         // degree 단위
static double k = 5;                           // 하이퍼파라미터
//static double x_error;                         // m단위
//static double path_angle = 0;                  // psi를 구하기 위한 각도로, 기준 좌표계로부터 생성된 경로(Path)사이의 기울기 각도
// double stanley_steering_angle = 0;      // 최종 출력값
// 현재차량속도 : current_vehicle_speed    // 단위는 안맞춰줘도 됨. (k 파라미터 때문)
// 해딩 앵글 : dr_yaw_angle

PD_CONTROL pd_control;
//TSR *tsr;

#ifdef STEERING_ANGLE_MOVING_AVG
    // Moving average tmp init
//    double str_moving_avg_tmp1 = 0;
//    double str_moving_avg_tmp2 = 0;
//    double str_moving_avg_tmp3 = 0;
//    double str_moving_avg_result = 0;

#define str_moving_avg_num 1500
    double str_moving_avg_tmp[str_moving_avg_num] = {};
    //double str_moving_avg_result = 0;

#endif

void* path_follow(void *flag)
{
//    dr_x = 32;    // 지워야함
    while(stanley_steering_control_enable_flag == 1)
    {

        // 추종 waypoint path를 바꿔주는 코드
        waypoint_change();
        
        // target_vehicle_speed = coord_map[i].WP_target_speed
        
        /////////////////////////////////////
        // 추종 waypoint를 이용한 path follow //
        /////////////////////////////////////
        
        // Longitude Control
        longitude_control();
        
        // Lateral Control
        stanley_steering_control();
        
        //std::cout << "Biggest one is " << TSR_detected_num << std::endl;
        //std::cout << "Classifier result :[" << TSR_detected_num << "] :" << tsr[TSR_detected_num].data_class << "\t\t" << tsr[TSR_detected_num].data_probability << "\t\t" << tsr[TSR_detected_num].data_xmin << "\t\t" << tsr[TSR_detected_num].data_xmax << "\t\t" << tsr[TSR_detected_num].data_ymin <<  "\t\t" << tsr[TSR_detected_num].data_ymax << "\t\t" << tsr[TSR_detected_num].roi_size << std::endl;

        // CPU 과부하 방지
        //usleep(20000);
        usleep(1);
    }
    
//    return 0;
}

void longitude_control()
{

    target_vehicle_speed = coord_map[coord_current_address].WP_target_speed;

    //    if(vehicle_yaw_rate_error_correct == 1)
//        std::cout << "target_vehicle_speed = " << target_vehicle_speed << "\t\tvehicle_speed =" << current_vehicle_speed << std::endl;
    
    // S_t, K_p, K_d는 상수, S_r은 실시간 CAN receive data. S_r에 뮤텍스(mutex) 들어갈 수 도 있음.
    if(speed_pid_control_enable_flag == 1)
    {
        aReqMax_Cmd = pd_control.MV_Cal_Func(target_vehicle_speed, current_vehicle_speed, K_p, K_d);
        aReqMax_Cmd = aReqMax_Cmd > 5 ? 5 : aReqMax_Cmd;
        aReqMax_Cmd = aReqMax_Cmd < -5 ? -5 : aReqMax_Cmd;
    }
}






void waypoint_change()
{
    // Rover좌표(DR_x, DR_y)와 WayPoint좌표(coord_map[address].x, coord_map[address].y)
    Rover2WP_distance = calc_two_point_distance(coord_map[coord_current_address].x, coord_map[coord_current_address].y, dr_x, dr_y);

    ////////////////////////////////////////////////
    // 얘 조건 수정해야함.
    //      차속이 빠르면 카운팅이 안됨.
    ////////////////////////////////////////////////
    ////////////////////////////////////////////////
    ////////////////////////////////////////////////
    /*
        헤딩과
        차량과 좌표의 각 ( GNSS기준 ) -> 좌표계 이동
     cos(3.1415926 / 180. * dr_yaw_angle)
     */
    double theta_tmp;
    theta_tmp = dr_yaw_angle;
    double x_tmp = cos(3.1415926 / 180. * -theta_tmp) * (coord_map[coord_current_address].x - dr_x) - sin(3.1415926 / 180. * -theta_tmp) * (coord_map[coord_current_address].y - dr_y);
    double y_tmp = sin(3.1415926 / 180. * -theta_tmp) * (coord_map[coord_current_address].x - dr_x) + cos(3.1415926 / 180. * -theta_tmp) * (coord_map[coord_current_address].y - dr_y);
    
    double angle_tmp = abs(atan2(y_tmp, x_tmp) * 180 / 3.1415926);
    
    ////////////////////////////////////////////////
    
//    std::cout << "angle_tmp = " << angle_tmp << std::endl;
    // Rover와 Waypoint의 거리가 1m보다 짧으면 다음 Waypoint로 이동
//    if(Rover2WP_distance < 1.5) // 1은 1m
    if((  (Rover2WP_distance < 3.5)&&(angle_tmp > 80))) //(Rover2WP_distance < 0.5) ||
    {
        coord_current_address++;
    }
    
//    std::cout << "Rover2WP_distance = " << Rover2WP_distance << std::endl;
//    std::cout << "coord_current_address = " << coord_current_address << std::endl;
    
    
#ifdef OpenCV_View_MAP
    //        img_coord_map.data[(int)(coord_map[i].y*4) * img_coord_map.cols + (int)(coord_map[i].x*4)] = 0;

    // viewer에 그리기
    cv::Point dot(coord_map[coord_current_address].x*4, coord_map[coord_current_address].y*4);
//    cv::circle(img, dot, 1, CvScalar(0,0,255));

    // 좌표지도용 좌표이미지에 그리기
//    cv::imshow("Coordinate_map", img);
    
    // 실시간 좌표이미지에 그리기
    cv::circle(img, dot, 3, CvScalar(0,0,255));

    //        cv::imshow("Coordinate_map", img);
//    cv::waitKey(1);
#endif
    
}








void stanley_steering_control()
{
    /*
     stanley 구현
     */
    
    
    // 필요한 변수 : x_error, psi, k, current_vehicle_speed, heading(dr_yaw_angle), path_angle
    
    
    // input : x(t), psi(heading - x와 수직인 애(=path기울기)), 차량속도, k변수
    
    
    
    // 스티어앵글각도   = 프사이(t) + arctan(k * x(t) / 차량속도)
    //              = heading - path_angle + arctan(k * x(t) / 차량속도)
    //              스티어앵글각도는 최대 45도라고 가정한다. -> 1값당 0.09도
    
    
    
//    // stanley 필요한 변수
//    double psi = 0;                         // degree 단위
//    double k = 1;                           // 하이퍼파라미터
//    double x_error;                         // m단위
//    double path_angle = 0;                  // psi를 구하기 위한 각도로, 기준 좌표계로부터 생성된 경로(Path)사이의 기울기 각도
//    double stanley_steering_angle = 0;      // 최종 출력값
//    // 현재차량속도 : current_vehicle_speed    // 단위는 안맞춰줘도 됨. (k 파라미터 때문)
//    // 해딩 앵글 : dr_yaw_angle
    
    
    
    
    /*
     stanley의 x_error 구하기
     */
    
    // 헤론의 공식을 이용해 삼각형 넓이 구하기
    triangle_area = 0;
    distance_a = calc_two_point_distance(dr_x, dr_y, coord_map[coord_current_address - 1].x, coord_map[coord_current_address - 1].y);
    distance_b = calc_two_point_distance(coord_map[coord_current_address - 1].x, coord_map[coord_current_address - 1].y, coord_map[coord_current_address].x, coord_map[coord_current_address].y);
    distance_c = calc_two_point_distance(dr_x, dr_y, coord_map[coord_current_address].x, coord_map[coord_current_address].y);
    
    s = (distance_a + distance_b + distance_c)/2;

    double area_tmp = s * (s - distance_a) * (s - distance_b) * (s - distance_c);
    area_tmp = area_tmp < 0 ? -area_tmp : area_tmp;

    triangle_area = sqrt(area_tmp);
    
    // 삼각형 넓이를 통해 x(오차) 구하기
    x_error = 2 * triangle_area / distance_b;

    //if(x_error > 20)
    //    x_error = 0;
    
    
    
    
    /*
     psi 구하기
     */
    
    // (0, 0), (axis_mv_x, axis_mv_y)로 path_angle 구하기
    double axis_mv_x = coord_map[coord_current_address].x - coord_map[coord_current_address - 1].x;
    double axis_mv_y = coord_map[coord_current_address].y - coord_map[coord_current_address - 1].y;
    
    path_angle = atan2(axis_mv_y, axis_mv_x) * 180 / 3.1415926; // tan2써야 360도 표현 가능.
    
    
//    dr_yaw_angle = 0;      //지워야함
//    path_angle = 90;        //지워야함
    psi = path_angle - dr_yaw_angle;    // Path와 평행이 되도록 vehicle steer angle을 맞춰준다.
                                        // 따라서 경우에 따라 +, - 모두 가능.
//    psi = path_angle - gnss_yaw_angle;    // 실제로 차량의 요 레이트 앵글을 넣어야하는데, 현재 스티어링이 들어가있다.
                                        // 즉, 스티어링이 변화하면서 차량이 실제로 움직이지 않았는대
                                        //      헤딩이 움직이는 결과를 가져와서 결과가 계속 변하므로 우선은 gnss 헤딩을 넣는다.

    
    psi = psi > 180 ? (360-psi) : psi;
    psi = psi < -180 ? (360 + psi) : psi;
    
    
    
    // 스티어앵글각도   = 프사이(t) + arctan(k * x(t) / 차량속도)
    // 오른쪽 = 마이너스
    // 왼쪽 = 플러스
    // stanley_steering_angle을 +로할지 -로할지 정해야함.
    
    
//    current_vehicle_speed = 100;     //지워야함
//    psi = 10;                       //지워야함
//    k = 1;
    //k = 3.5;        // 7kph에서 gnss_receive_data_9_18_3.txt 주행 가능.
    k = 0.5;         // Straight
    //k = 0.5;        // 수정 후 k값 (위 3.5는 수정 전)
//    k = 8;
//    // 각도별 앵글값 출력
//    for(int i = 0; i <= 360; i++)
//    {
//        psi = path_angle - dr_yaw_angle;    // Path와 평행이 되도록 vehicle steer angle을 맞춰준다.
//        double atan_tmp =atan(k * x_error / current_vehicle_speed) * 180 / 3.1415926;
//        int direction_tmp = rover_steering_angle_direction();          // 차량(Rover)가 진행 방향의 좌측인지, 우측인지 구분하기 위한 함수
//        atan_tmp *= (double)direction_tmp;
//        stanley_steering_angle = psi + atan_tmp;
//        std::cout << "stanley_steering_angle = " << stanley_steering_angle << "\t\tdr_yaw_angle = " << dr_yaw_angle << "\t" << stanley_steering_angle + dr_yaw_angle << std::endl;
//        dr_yaw_angle++;
//    }

    
    // x_error를 cm단위로 변환 후, 제곱해주어 멀어질수록 더 각변화를 심화시킴
//    double atan_tmp =atan(k/10000 * (x_error * 100) * (x_error * 100)  / (current_vehicle_speed+5)) * 180 / 3.1415926;
    double offset_speed = 5; // 기존엔 5였음.
    atan_tmp =atan(k * x_error / (current_vehicle_speed + offset_speed)) * 180 / 3.1415926;
    int direction_tmp = rover_steering_angle_direction();          // 차량(Rover)가 진행 방향의 좌측인지, 우측인지 구분하기 위한 함수
    atan_tmp *= (double)direction_tmp;
    stanley_steering_angle = -(psi + atan_tmp);
    

    // stanley_steering_angle이 음수면 양수로 바꿔주기
//    stanley_steering_angle = stanley_steering_angle < 0 ? 360+stanley_steering_angle : stanley_steering_angle;
    
    // heading값 빼주기
//    stanley_steering_angle -= gnss_yaw_angle;
    
//    std::cout << "gnss_yaw_angle = " << gnss_yaw_angle << std::endl;
    
    if(vehicle_yaw_rate_error_correct == 1)
    {
#ifdef STEERING_ANGLE_MOVING_AVG
//        str_moving_avg_tmp1 = str_moving_avg_tmp2;
//        str_moving_avg_tmp2 = str_moving_avg_tmp3;
//        str_moving_avg_tmp3 = stanley_steering_angle;

        for(int i = 0; i < str_moving_avg_num-1; i++)	// 0 1 2 3
        {
            str_moving_avg_tmp[i] = str_moving_avg_tmp[i+1];
            //std::cout << "tmp = " << str_moving_avg_tmp[i] << std::endl;
        }
        str_moving_avg_tmp[str_moving_avg_num-1] = stanley_steering_angle;

//        std::cout << psi << "\t1\t" << atan_tmp << "\t\t" << current_vehicle_speed << std::endl;
//        std::cout << stanley_steering_angle << std::endl;
//        std::cout << "===============================================" << std::endl;
//        str_moving_avg_tmp[0] = str_moving_avg_tmp[1];
//        str_moving_avg_tmp[1] = str_moving_avg_tmp[2];
//        str_moving_avg_tmp[2] = str_moving_avg_tmp[3];
//        str_moving_avg_tmp[3] = str_moving_avg_tmp[4];
//        str_moving_avg_tmp[4] = stanley_steering_angle;

        str_moving_avg_result = 0;

        for(int i = 0; i < str_moving_avg_num; i++)
        {
            str_moving_avg_result += (str_moving_avg_tmp[i] / str_moving_avg_num);
        }

        //str_moving_avg_result = str_moving_avg_result / str_moving_avg_num;

//        str_moving_avg_result = (str_moving_avg_tmp[0] + str_moving_avg_tmp[1] + str_moving_avg_tmp[2] + str_moving_avg_tmp[3] + str_moving_avg_tmp[4]) / 5;

        steer_angle = str_moving_avg_result * 500/30;

        // 클리핑
        steer_angle = steer_angle > 500 ? 500 : steer_angle;
        steer_angle = steer_angle < -500 ? -500 : steer_angle;
#elif
        // 스티어링 1도에 바퀴 0.06도 회전으로 계산.
        steer_angle = stanley_steering_angle * 500/30;

        // 클리핑
        steer_angle = steer_angle > 500 ? 500 : steer_angle;
        steer_angle = steer_angle < -500 ? -500 : steer_angle;
#endif
    }


#ifdef LATERAL_CONTROL_PRINT
    std::cout << stanley_steering_angle << "\t\t" << steer_angle << std::endl;
#endif
    // 추종 각도 (동일한 x_error, 속도에서 차량의 헤딩이 변해도 경로를 추종하기 위해 접근하는 각도가 동일하다.)
    double follow_angle = stanley_steering_angle + dr_yaw_angle;
    //    std::cout << "follow angle : " << follow_angle << std::endl;
//    std::cout << "x_error : " << x_error << std::endl;
//    std::cout << "stanley_steering_angle = " << stanley_steering_angle << std::endl;
//    std::cout << "distance_a = " << distance_a << std::endl;
//    std::cout << "distance_b = " << distance_b << std::endl;
//    std::cout << "distance_c = " << distance_c << std::endl;
//    std::cout << "path_angle = " << path_angle <<"\t\tpsi = " << psi  << "\t\tatan_tmp = " << atan_tmp << "\t\tst_angle = " << stanley_steering_angle << "\t\tx_error = " << x_error << std::endl;
//    std::cout << "atan_tmp = " << atan_tmp << std::endl;
//    std::cout << "gnss_yaw_angle = " << gnss_yaw_angle << std::endl;

    
    
    int avsdcds;
}



int rover_steering_angle_direction()
{
    // translation, rotation작업을 통해 좌표지도의 출발 waypoint (0,0)에 놓고 끝 waypoint를 y=0 축에 올린 후, rover의 좌표 중 y가 +인지, -인지를 판단하여 stanley_steering_angle이 +인지, -인지를 계산하는 방법.
    double dr_x_axis_mv = dr_x - coord_map[coord_current_address - 1].x;
    double dr_y_axis_mv = dr_y - coord_map[coord_current_address - 1].y;
    double dr_y_axis_mv_rotation = sin(3.1415926 / 180. * -path_angle) * dr_x_axis_mv + cos(3.1415926 / 180. * -path_angle) * dr_y_axis_mv;
    
    if(dr_y_axis_mv_rotation > 0)   // 진행 방향의 왼쪽에 있음.
        return -1;
    else                            // 진행 방향의 오른쪽에 있음.
        return 1;
}




double calc_two_point_distance(double x_1_, double y_1_, double x_2_, double y_2_)
{
    double two_point_distance_ = 0;
    double calc_tmp_ = (x_2_ - x_1_) * (x_2_ - x_1_) + (y_2_ - y_1_) * (y_2_ - y_1_);
    calc_tmp_ = calc_tmp_ < 0 ? -calc_tmp_ : calc_tmp_;

    two_point_distance_ = sqrt(calc_tmp_);
    return two_point_distance_;
}

