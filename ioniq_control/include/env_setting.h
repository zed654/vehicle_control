// GNSS Map Viewer
#define OpenCV_View_MAP

#define APM_En            (unsigned char)0x01
#define APM_D_En          (unsigned char)0x00
#define ASM_En            (unsigned char)0x02
#define ASM_D_En          (unsigned char)0x00
#define APM_IGNORE        (unsigned char)0x4
#define ACC_StopReq_En    (unsigned char)0x01
#define ACC_StopReq_D_En  (unsigned char)0x00

#ifdef __APPLE__


// CAN device env setting
#define CONTROL_CAN_DEVICE PCAN_USBBUS1
#define CHASSIS_CAN_DEVICE PCAN_USBBUS2
#define PCAN_BAUD_RATE PCAN_BAUD_500K

// GNSS device env setting
#define GNSS_PORT_PATH "/dev/tty.usbmodem1442210"
#define GNSS_BAUD_RATE 9600


#else   //__APPLE__


// CAN device env setting
//      sudo modprobe can
//      sudo ip link set can0 type can bitrate 500000
//      sudo ip link set can1 type can bitrate 500000
//      sudo ip link set can0 up
//      sudo ip link set can1 up
#define CONTROL_CAN_DEVICE "can1"
#define CHASSIS_CAN_DEVICE "can0"
#define PCAN_BAUD_RATE 0  // have not to define

// GNSS device env setting
//      sudo chmod 777 /dev/ttyACM0
#define GNSS_PORT_PATH "/dev/ttyACM0"
#define GNSS_BAUD_RATE 9600


#endif  // __APPLE__

// Thread enable definition (CAN, GNSS)
//#define CAN_CONTROL_ENABLE
//#define CAN_CHASSIS_ENABLE
//#define GNSS_ENABLE

//GNSS data write
#define GNSS_DATA_WRITE








////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////








// print_ CAN running time
extern bool CAN_running_time_flag;

// print_ control CAN flag
extern bool print_can_control_state_flag;
extern bool print_vehicle_speed_flag;
extern bool print_alive_count_flag;
extern bool print_steer_angle_flag;
extern bool print_cluster_speed_display_flag;

// print_ chassis CAN flag
extern bool print_driving_tq_flag;
extern bool print_cluster_engine_rpm_flag;
extern bool print_new_msg_engine_rpm_flag;         // motor rpm인듯
extern bool print_rpm_without_electro_flag;        // engine rpm인듯
extern bool print_brake_pressure_flag;
extern bool print_whl_spd_flag;
extern bool print_whl_pul_flag;
extern bool print_yaw_rate_flag;

// APM, ASM enable state
extern unsigned char APM_state;   // APM_D_En, APM_En
extern unsigned char ASM_state;   // ASM_D_En, ASM_En

// Control param
extern unsigned int APM_Slevel_val;  // [100, 250], (if this value set to 0, APM set to 150)
extern int steer_angle;    // 0x14 // value * 10 => 0xc8 // [-500, 500]
extern float aReqMax_Cmd;       // -5.00 ~ 5.00 소수점 2째 자리까지 가능
extern int cluster_speed_display_value;

// Stanley Steering Control
extern bool stanley_steering_control_enable_flag;

// PD control param
extern bool speed_pid_control_enable_flag;
extern int target_vehicle_speed;             // 타겟 속도
extern int current_vehicle_speed;               // 현재 속도
extern float K_p;
extern float K_d;

// 0.2, 0.5 -> 15

// Dead Reckoning
extern double vehicle_yaw_rate;
extern double vehicle_steering_angle;
extern double yaw_rate_accum_tmp;

// CAN error flag (전역으로 선언)
extern bool can_error_flag;
// GNSS error flag
extern bool gnss_error_flag;
// DR(Dead Reckoning) error flag
extern bool DR_error_flag;



// GNSS, Dead Reckoning variable
// Starting point   : 36.72790320        127.44262740
//      기준의 x, y좌표(m단위)
extern double base_latitude;
extern double base_longitude;

extern double gnss_x;
extern double gnss_y;
extern double gnss_yaw_angle;
extern double GNSS_heading;

// Dead Reckoning 결과
extern double dr_yaw_angle;
extern double dr_yaw_rate_integral_val;
extern double dr_x;
extern double dr_y;
extern double delta_dr_x;
extern double delta_dr_y;

extern bool gnss_initial_flag;

extern int count_dr;
extern int count_gnss;

extern double dr_hard_x;
extern double dr_hard_y;

// Yar rate 초기 애러 보정용 변수
extern int yaw_rate_count;
extern double yaw_rate_accum_value;
extern double yaw_rate_error_mean_value;
extern bool vehicle_yaw_rate_error_correct;    // 애러 보정이 완료됬다는 플래그

// CAN alive counter
extern unsigned char CAN_alive_count;

// Path follow
extern int coord_current_address;
extern double str_moving_avg_result;
extern double stanley_atan;
extern double stanley_steering_angle;      // 최종 출력값
extern double psi;                         // degree 단위
extern double s;
extern double x_error;
extern double path_angle;                  // psi를 구하기 위한 각도로, 기준 좌표계로부터 생성된 경로(Path)사이의 기울기 각도
extern double triangle_area;
extern double distance_a;
extern double distance_b;
extern double distance_c;


#ifdef OpenCV_View_MAP
// This map is fitted for Ochang PG
#include <opencv2/opencv.hpp>
extern cv::Mat img;
extern double opencv_x;
extern double opencv_y;
#endif
