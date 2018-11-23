/*

    필수!!!!!!! CAN 연결 순서 체크하고 진행하기
    (제어캔, 샤시캔 구분하는 방법 찾아서 캔 연결 순서를 무관하게 만들어보기)

 */

/*
    -- Steering Control ---
    stanley                                     // 고속에서는 무리

    mpc = model predictive control              // 중요! .. kaist
    adaptive pure pursuit                       // 중요! .. kaist
    anchorman steering model                    // bicyle model로
 */

// GNSS Map Viewer
#include "env_setting.h"

#ifdef OpenCV_View_MAP
#include <opencv2/opencv.hpp>
#endif

#include <iostream>
#include <pthread.h>
#include <fstream>
#include <sys/time.h>
#include <math.h>       // abs()
#include <stdlib.h>     // _abs64()
#include <unistd.h>
//#include "can_mac_val.h"
//#include "pd_control.h"
#include "sys/time.h"
#include "parser_vercpp.hpp"
#include "coord_map.h"

#ifdef OpenCV_View_MAP
// This map is fitted for Ochang PG
//cv::Mat img(700, 300, CV_8UC3);

//K-city
//cv::Mat img(700, 1800, CV_8UC3);

// Ochang
cv::Mat img(700, 300, CV_8UC3);

double opencv_x = 0;
double opencv_y = 0;
#endif

// 좌표지도 좌표 저장 변수를 전역으로 선언 (coord_map.cpp에서 접근)
//extern coordinates_map *coord_map;
coordinates_map *coord_map;

// print_ CAN running time
bool CAN_running_time_flag = 0;

// print_ control CAN flag
bool print_can_control_state_flag = 0;
bool print_vehicle_speed_flag = 0;
bool print_alive_count_flag = 0;
bool print_steer_angle_flag = 0;
bool print_cluster_speed_display_flag = 1;

// print_ chassis CAN flag
bool print_driving_tq_flag = 0;
bool print_cluster_engine_rpm_flag = 0;
bool print_new_msg_engine_rpm_flag = 0;         // motor rpm인듯
bool print_rpm_without_electro_flag = 0;        // engine rpm인듯
bool print_brake_pressure_flag = 0;
bool print_whl_spd_flag = 0;
bool print_whl_pul_flag = 0;
bool print_yaw_rate_flag = 0;

// APM, ASM enable state
unsigned char APM_state = APM_En;   // APM_D_En, APM_En
unsigned char ASM_state = ASM_En;   // ASM_D_En, ASM_En

// Control param
unsigned int APM_Slevel_val = 250;  // [100, 250], (if this value set to 0, APM set to 150)
int steer_angle = 0;    // 0x14 // value * 10 => 0xc8 // [-500, 500]
float aReqMax_Cmd = -1.;       // -5.00 ~ 5.00 소수점 2째 자리까지 가능
int cluster_speed_display_value = 1;

// Stanley Steering Control
bool stanley_steering_control_enable_flag = 1;

// PD control param
bool speed_pid_control_enable_flag = 1;
int target_vehicle_speed = 0;             // 타겟 속도
int current_vehicle_speed = 0;               // 현재 속도
float K_p = 0.05;
float K_d = 0.5;

// 0.2, 0.5 -> 15

// Dead Reckoning
double vehicle_yaw_rate = 0;
double vehicle_steering_angle = 0;

// CAN error flag (전역으로 선언)
bool can_error_flag = 0;
// GNSS error flag
bool gnss_error_flag = 0;
// DR(Dead Reckoning) error flag
bool DR_error_flag = 0;



// GNSS, Dead Reckoning variable
// Starting point   : 36.72790320        127.44262740
// K-City           : 37.24385200		126.77555800
//      기준의 x, y좌표(m단위)
//double base_latitude = 37.24385200;//36.72790320;
//double base_longitude = 126.77555800;//127.44262740;
double base_latitude = 36.72790320;
double base_longitude = 127.44262740;
double gnss_x = 0;
double gnss_y = 0;
double gnss_yaw_angle = 0;
double GNSS_heading = 0;

// Dead Reckoning 결과
double dr_yaw_angle = 0;
double dr_yaw_rate_integral_val = 0;
double dr_x = 0;
double dr_y = 0;
double delta_dr_x = 0;
double delta_dr_y = 0;
double yaw_rate_accum_tmp = 0;

bool gnss_initial_flag = 0;

int count_dr = 0;
int count_gnss = 0;

double dr_hard_x = 28.14;
double dr_hard_y = 9.86;

// Yar rate 초기 애러 보정용 변수
int yaw_rate_count = 0;
double yaw_rate_accum_value = 0;
double yaw_rate_error_mean_value = 0;
bool vehicle_yaw_rate_error_correct = 0;    // 애러 보정이 완료됬다는 플래그

// Path follow
int coord_current_address = 1;
double str_moving_avg_result = 0;
double atan_tmp = 0;
double stanley_steering_angle = 0;      // 최종 출력값
double psi = 0;                         // degree 단위
double s = 0;
double x_error = 0;
double path_angle = 0;                  // psi를 구하기 위한 각도로, 기준 좌표계로부터 생성된 경로(Path)사이의 기울기 각도
bool vehicle_restart_flag = 0;
double triangle_area = 0;
double distance_a = 0;
double distance_b = 0;
double distance_c = 0;

// pthread function
void* CAN_RW(void *can_error_flag_);
void* CCAN_RW(void *can_error_flag_);
void* GNSS_Receive(void *gnss_error_flag_);
void* Dead_Reckoning(void *DR_error_flag_);
void* path_follow(void *flag);
void* Print_Write_thread(void *param);

// Read Coordinates Map Data
void Coordinates_Map_Data();

// CAN alive counter
//static BYTE CAN_alive_count = 0;
unsigned char CAN_alive_count = 0;

// mutex
//pthread_mutex_t  mutex = PTHREAD_MUTEX_INITIALIZER; // 쓰레드 초기화








//typedef struct ARG
//{
//    int argc;
//    char** argv;
//} ARG;

//ARG *arg;

//#include <ros/ros.h>
//#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <darknet_ros_msgs/BoundingBox.h>
//// TSR Callback function
//void TSR_MsgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);

int main(int argc, char** argv)
{

#ifdef OpenCV_View_MAP
    img.setTo(0);
#endif

    // Read Coordinates Map Data
    Coordinates_Map_Data();

//    PD_CONTROL pd_control;

    pthread_t p_thread[6];

//#if defined(CAN_CONTROL_ENABLE) || defined(CAN_CHASSIS_ENABLE)
//    // CAN Read Write
//    pthread_create(&p_thread[0], NULL, CAN_RW, (void *)can_error_flag);
//    pthread_create(&p_thread[1], NULL, CCAN_RW, (void *)can_error_flag);
//#else
//    printf("CAN_CONTROL_ENABLE and CAN_CHASSIS_ENABLE is not defiend by #define comment\n\n");
//#endif

    printf("\n");
#ifdef CAN_CONTROL_ENABLE
    //CAN Read Write
    pthread_create(&p_thread[0], NULL, CAN_RW, (void *)can_error_flag);
#else
    printf("CAN_CONTROL_ENABLE is not defined by #define comment\n");
#endif

#ifdef CAN_CHASSIS_ENABLE
    //CAN Read Write
    pthread_create(&p_thread[1], NULL, CCAN_RW, (void *)can_error_flag);
#else
    printf("CAN_CHASSIS_ENABLE is not defined by #define comment\n");
#endif

#ifdef GNSS_ENABLE
    // GNSS Receive
    pthread_create(&p_thread[2], NULL, GNSS_Receive, (void *)gnss_error_flag);//(void *)multiple_arg);
#else
    printf("GNSS_ENABLE is not defined by #define comment\n\n");
#endif

    // DR thraed
    pthread_create(&p_thread[3], NULL, Dead_Reckoning, (void *)DR_error_flag);

    // Waypoint Path change, Stanley steering control(Lateral Control), Target Point follow PD control(longitudinal control)
    pthread_create(&p_thread[4], NULL, path_follow, NULL);

    pthread_create(&p_thread[5], NULL, Print_Write_thread, NULL);


    while(1)
    {
#ifdef OpenCV_View_MAP
        std::cout << std::fixed;
        std::cout.precision(4);
//        std::cout << opencv_x << "\t\t" << opencv_y << std::endl;
//        pthread_mutex_lock(&mutex); // 잠금을 생성한다.
//        cv::Point dot(opencv_x, opencv_y);
//        cv::circle(img, dot, 1, CvScalar(0,255,255));
//        pthread_mutex_unlock(&mutex); // 잠금을 해제한다.
        cv::imshow("Coordinate", img);
        if(cv::waitKey(1) == 'q')  break;
//        printf("main loop\n");
#endif

        /*
         차량 제어 코드 작성
         */

//        // S_t, K_p, K_d는 상수, S_r은 실시간 CAN receive data. S_r에 뮤텍스(mutex) 들어갈 수 도 있음.
//        if(speed_pid_control_enable_flag == 1)
//        {
//            aReqMax_Cmd = pd_control.MV_Cal_Func(target_vehicle_speed, current_vehicle_speed, K_p, K_d);
//            aReqMax_Cmd = aReqMax_Cmd > 5 ? 5 : aReqMax_Cmd;
//            aReqMax_Cmd = aReqMax_Cmd < -5 ? -5 : aReqMax_Cmd;
//        }

        usleep(1);
        if(can_error_flag == true)  break;
    }




    //    pthread_mutex_destroy(&mutex);
    //    pthread_join(p_thread[0], (void **) &status);
    pthread_join(p_thread[0], NULL);
    pthread_join(p_thread[1], NULL);
    pthread_join(p_thread[2], NULL);
    pthread_join(p_thread[3], NULL);
    pthread_join(p_thread[4], NULL);
    pthread_join(p_thread[5], NULL);
    return 0;
}
//	  	<arg name="video_stream_provider" value="/home/chp/darknet_ros_ws/src/darknet_ros/darknet_ros/doc/airport.avi" />

//#define YAW_ANGLE_WRITE
#define __PRINT__

void* Print_Write_thread(void *param)
{
    
#ifdef YAW_ANGLE_WRITE
    std::ofstream fin;

#ifdef __APPLE__
    fin.open("../../YAW_ANGLE/yaw_angle_save.txt");
#else
    fin.open("/home/chp/darknet_ros_ws/src/darknet_ros/ioniq_control/YAW_ANGLE/yaw_angle_save.txt");
#endif
    
    fin << std::fixed;
    fin.precision(8);
    fin << "GNSS heading" << "\t\t" << "dr_yaw_angle" << "\t\t" << "yaw_rate_accum_tmp" << "\t\tsteer_angle" << "\t\tstr_moving_avg_result" << "\t\tstanley_steering_angle" << "\t\tpath_angle" << "\t\tdr_yaw_angle" << "\t\tpsi" << "\t\tatan_tmp" << "\t\ts" << "\t\ttriangle_area" <<  "\t\tdistance_a" << "\t\tdistance_b" << "\t\tdistance_c" << "\t\tx_error" << std::endl;
#endif
    
    while(1)
    {
        
#ifdef YAW_ANGLE_WRITE
    if(vehicle_yaw_rate_error_correct == 1)
    {
        fin << std::fixed;
        fin.precision(8);
        fin << GNSS_heading << "\t\t" << dr_yaw_angle << "\t\t" << yaw_rate_accum_tmp << "\t\t" << steer_angle << "\t\t" << str_moving_avg_result  << "\t\t" << stanley_steering_angle << "\t\t" << path_angle << "\t\t" << dr_yaw_angle << "\t\t" << psi << "\t\t" << atan_tmp << "\t\t" << s << "\t\t" << triangle_area << "\t\t" << distance_a << "\t\t" << distance_b << "\t\t" << distance_c << "\t\t" << x_error << std::endl;
        std::cout << GNSS_heading << "\t\t" << dr_yaw_angle << "\t\t" << yaw_rate_accum_tmp << "\t\t" << steer_angle << std::endl;
    }
#endif

/*
#ifdef __PRINT__
        if(vehicle_yaw_rate_error_correct == 1)
        {
            std::cout << "current_SP_distance : " << current_SP_distance << "\t\tcurrent_address : " << coord_current_address << "\t\tSP_passing_flag : " << SP_passing_flag <<
                     "\t\tinit_SP_distance : " << init_SP_distance << "\t\tinit_target_speed : " << init_target_speed << std::endl;
            std::cout << "current_vehicle_speed : " << current_vehicle_speed << "\t\ttarget_speed : " << target_vehicle_speed << std::endl;
        }
#endif
*/
/*
 * #ifdef __PRINT__
    if(vehicle_yaw_rate_error_correct == 1)
    {
        std::cout << "current_SP_distance : " << current_SP_distance << "\t\tcurrent_address : " << coord_current_address << "\t\tSP_cross_flag : " << SP_cross_flag <<
                 "\t\tinit_SP_distance : " << init_SP_distance << "\t\tinit_target_speed : " << init_target_speed << std::endl;
        std::cout << "current_vehicle_speed : " << current_vehicle_speed << "\t\ttarget_speed : " << target_vehicle_speed << std::endl;
    }
#endif
*/
        usleep(1000);
        //usleep(1);
    }
}
