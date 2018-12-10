
/*
    CAN chassis, CAN control 의 데이터 Read/Write 스레드.
 */

////#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include "env_setting.h"
#ifdef __APPLE__
#include "can_mac_val.h"
#else
#include "can_linux_val.h"
#endif

////
// Control CAN
void Mo_Conf(CAN_AVL &can_);
void Mo_Val(CAN_AVL &can_);
bool Report_APM(CAN_AVL &can_);
bool Report_ASM(CAN_AVL &can_);
bool Report_Misc(CAN_AVL &can_);
void can_print(CAN_AVL can_);

// Chassis CAN
void TCU_DVT13_Driving_Tq(CAN_AVL can_);            // ID : 0x162, LEN : 3
void TCU_DCT13_Engine_RPM(CAN_AVL can_);            // ID : 0x162, LEN : 3
void NEW_MSG_ENGINE(CAN_AVL can_);                  // ID : 0x240, LEN : 8, rpm
void NEW_MSG_2_RPM_WITHOUT_ELECTRO(CAN_AVL can_);   // ID : 0x371, LEN : 8
void Brake_Pressure(CAN_AVL can_);                  // ID : 0x371, LEN : 8
void WHL_Speed(CAN_AVL can_);                       // ID : 0x386, LEN : 8, km/h
void WHL_PUL(CAN_AVL can_);                         // ID : 0x387, LEN : 6
void YAW_RATE(CAN_AVL can_);                        // ID : 0x220, LEN : 13, degree/s
////
//static BYTE CAN_alive_count = 0;
////
void* CAN_RW(void *can_error_flag_)
{
    printf("Init can_error_flag : %d\n", can_error_flag_);
#ifdef __APPLE__
    
#ifdef CAN_CONTROL_ENABLE
    CAN_AVL can_control(CONTROL_CAN_DEVICE, PCAN_BAUD_RATE);      // 제어 캔. write/read
    memset(&can_control.message, 0, sizeof(TPCANMsg));
    can_control.PEAKCAN_TO_SOCKETCAN();
#else   // CAN_CONTROL_ENABLE
    printf("CAN_CONTROL_ENABLE is not defined by #define comment\n\n");
#endif  // CAN_CONTROL_ENABLE
    
#else   // __APPLE__
    
#ifdef CAN_CONTROL_ENABLE
    CAN_AVL can_control(CONTROL_CAN_DEVICE, PCAN_BAUD_RATE);      // 제어 캔. write/read
    can_control.can_memset();
#else   // CAN_CONTROL_ENABLE
    printf("CAN_CONTROL_ENABLE is not defined by #define comment\n\n");
#endif  // CAN_CONTROL_ENABLE
    
#endif  // __APPLE__
    
//#ifdef CAN_CHASSIS_ENABLE
//    CAN_AVL can_chassis(CHASSIS_CAN_DEVICE, PCAN_BAUD_RATE);      // 샤시 캔. only read
//    memset(&can_chassis.message, 0, sizeof(TPCANMsg));
//    can_chassis.PEAKCAN_TO_SOCKETCAN();
//#else
//    printf("CAN_CHASSIS_ENABLE is not defined by #define comment\n\n");
//#endif

    // CAN R/W time check val
    struct timeval t1, t2;
    double elapsedTime = 0;

    while(1)
    {
        // start timer
        gettimeofday(&t1, NULL);

#ifdef CAN_CONTROL_ENABLE
        //////////////////////////////////////////////
        ////////////////  can_control  ///////////////
        //////////////////////////////////////////////

        // 초기화
        // 0x156 값넣고
        Mo_Conf(can_control);   // id 0x156
        // write

        // 초기화
        // 0x157 값 넣고
        Mo_Val(can_control);    // id 0x157
        // write

        //        can_control.can_memset();
        can_control.can_read();

        //        can_print(can_control);

        if(can_control.frame.can_id == 0x710)  can_error_flag = Report_APM(can_control);
        if(can_control.frame.can_id == 0x711)  can_error_flag = Report_ASM(can_control);
        if(can_control.frame.can_id == 0x71f)  can_error_flag = Report_Misc(can_control);

        //////////////////////////////////////////////
        //////////////////////////////////////////////
        //////////////////////////////////////////////
#endif






//#ifdef CAN_CHASSIS_ENABLE
//        //////////////////////////////////////////////
//        //////////////// can_chassis /////////////////
//        //////////////////////////////////////////////
//
//        //can_chassis.can_memset();
////        can_chassis.can_memset();
//        
//        
//        
//        can_chassis.can_read();
//
//        if(can_chassis.frame.can_id == 0x162) TCU_DVT13_Driving_Tq(can_chassis);    // 얜 write같음. test 해보진 말 것
//        if(can_chassis.frame.can_id == 0x162) TCU_DCT13_Engine_RPM(can_chassis);    // 얜 write같음. test 해보진 말 것
//        if(can_chassis.frame.can_id == 0x240) NEW_MSG_ENGINE(can_chassis);
//        if(can_chassis.frame.can_id == 0x371) NEW_MSG_2_RPM_WITHOUT_ELECTRO(can_chassis);
//        if(can_chassis.frame.can_id == 0x371) Brake_Pressure(can_chassis);
//        if(can_chassis.frame.can_id == 0x386) WHL_Speed(can_chassis);
//        if(can_chassis.frame.can_id == 0x387) WHL_PUL(can_chassis);
//        if(can_chassis.frame.can_id == 0x220) YAW_RATE(can_chassis);
//
//        //////////////////////////////////////////////
//        //////////////////////////////////////////////
//        //////////////////////////////////////////////
//#endif


        if((bool)can_error_flag_ == true)
            break;

//        if (vehicle_yaw_rate_error_correct == 1)
//            //usleep(20000);
//            usleep(1);
//        else
//            usleep(1);

        //        sleep(1);
        //        usleep(2000000);
        // usleep(1);
        //usleep(20000);

        // stop timer
        gettimeofday(&t2, NULL);

        // compute and print the elapsed time in millisec
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
        if(CAN_running_time_flag == 1) std::cout << elapsedTime << " ms.\n";

    }
    return 0; // 원래는 해줘야함. return (void *)value;
}

void* CCAN_RW(void *can_error_flag_)
{
    
    printf("Init can_error_flag : %d\n", can_error_flag_);

#ifdef __APPLE__
    
#ifdef CAN_CHASSIS_ENABLE
    CAN_AVL can_chassis(CHASSIS_CAN_DEVICE, PCAN_BAUD_RATE);      // 샤시 캔. only read
    memset(&can_chassis.message, 0, sizeof(TPCANMsg));
    can_chassis.PEAKCAN_TO_SOCKETCAN();
#else   // __CAN_CHASSIS_ENABLE
    printf("CAN_CHASSIS_ENABLE is not defined by #define comment\n\n");
#endif
    
#else   // __APPLE__
    
#ifdef CAN_CHASSIS_ENABLE
    CAN_AVL can_chassis(CHASSIS_CAN_DEVICE, PCAN_BAUD_RATE);      // 샤시 캔. only read
    can_chassis.can_memset();
#else   // CAN_CHASSIS_ENABLE
    printf("CAN_CHASSIS_ENABLE is not defined by #define comment\n\n");
#endif  // CAN_CHASSIS_ENABLE
    
#endif  // __APPLE__
    
    // CAN R/W time check val
    struct timeval t1, t2;
    double elapsedTime = 0;
    
    while(1)
    {
        // start timer
        gettimeofday(&t1, NULL);
        
#ifdef CAN_CHASSIS_ENABLE
        //////////////////////////////////////////////
        //////////////// can_chassis /////////////////
        //////////////////////////////////////////////
        
        //can_chassis.can_memset();
        //        can_chassis.can_memset();
        
        
        
        can_chassis.can_read();
        
        if(can_chassis.frame.can_id == 0x162) TCU_DVT13_Driving_Tq(can_chassis);    // 얜 write같음. test 해보진 말 것
        if(can_chassis.frame.can_id == 0x162) TCU_DCT13_Engine_RPM(can_chassis);    // 얜 write같음. test 해보진 말 것
        if(can_chassis.frame.can_id == 0x240) NEW_MSG_ENGINE(can_chassis);
        if(can_chassis.frame.can_id == 0x371) NEW_MSG_2_RPM_WITHOUT_ELECTRO(can_chassis);
        if(can_chassis.frame.can_id == 0x371) Brake_Pressure(can_chassis);
        if(can_chassis.frame.can_id == 0x386) WHL_Speed(can_chassis);
        if(can_chassis.frame.can_id == 0x387) WHL_PUL(can_chassis);
        if(can_chassis.frame.can_id == 0x220) YAW_RATE(can_chassis);
        
        //////////////////////////////////////////////
        //////////////////////////////////////////////
        //////////////////////////////////////////////
#endif
        
        
        if((bool)can_error_flag_ == true)
            break;
        
        // if use uSleep, Delay happen
        // if use uSleep, Delay happen
        // if use uSleep, Delay happen
        //        sleep(1);
        //        usleep(2000000);
//        usleep(1);

//        if (vehicle_yaw_rate_error_correct == 1)
//            //usleep(20000);
//            usleep(1);
//        else
//            usleep(1);

        // stop timer
        gettimeofday(&t2, NULL);
        
        // compute and print the elapsed time in millisec
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
        if(CAN_running_time_flag == 1) std::cout << elapsedTime << " ms.\n";
        
    }
    return 0; // 원래는 해줘야함. return (void *)value;
}



void Mo_Conf(CAN_AVL &can_)
{

    //    memset(can_.message.DATA, 0, sizeof(BYTE)*8);
    //    memset(&can_.message, 0, sizeof(TPCANMsg));
    //    can_.PEAKCAN_TO_SOCKETCAN();
    can_.can_memset();

    //    can_print(can_);

    unsigned long tmp_id = 0x156;
    unsigned char tmp_dlc = 8;
    can_.write_param(tmp_id, tmp_dlc);

    can_.frame.data[0] = APM_state;                  // APM_IGNORE, (APM_En / APM_D_En)
    can_.frame.data[1] = APM_Slevel_val * 0.5;      // APM_Slevel(Steering angular speed level) 0.5*(real_value), [0, 510]
    can_.frame.data[2] = ASM_state;                         // ASM_StopRequest(Req AEB action), ASM_En(En : 0x10, D_En : 0x00)
    can_.frame.data[3] = 0;
    can_.frame.data[4] = 0;
    can_.frame.data[5] = 0;
    can_.frame.data[6] = 0;
    can_.frame.data[7] = CAN_alive_count++;             // alive-count    can.write();

    //    can_print(can_);

//    printf("ID : %0.3x \t DLC : %d \t DATA : ", tmp_id, tmp_dlc);
//    for(int i = 0; i < 8; i++)
//        printf("%0.2x ", can_.frame.data[i]);
////    printf("\n");
    
        can_.can_write();
}


void Mo_Val(CAN_AVL &can_)
{
    can_.can_memset();      // data만 memset

    unsigned long tmp_id = 0x157;
    unsigned char tmp_dlc = 8;

    can_.write_param(tmp_id, tmp_dlc);

    can_.frame.data[0] = (u_int8_t)(((10*steer_angle) & 0x00ff));
    // steer_angle    Andgle of Steering      10 * (real_value)   [-500, 500]
    can_.frame.data[1] = (u_int8_t)((((10*steer_angle) & 0xff00) >> 8));
    // steer_angle
    can_.frame.data[2] = cluster_speed_display_value;
    //can_.message.DATA[2] = (u_int8_t)10 & 0x00ff;
    can_.frame.data[3] = (u_int8_t)((u_int16_t)(100 * (aReqMax_Cmd + 10.23)) & 0x00ff);
    // aReqMax_Cmd  Acceleration Control    100*((value)+10.23) [-5.0, 5.0]     멈추려면 -값 주면 된다고 함.
    can_.frame.data[4] = (u_int8_t)(((u_int16_t)(100 * (aReqMax_Cmd + 10.23)) >> 8) & 0x00ff);
    // aReqMax_Cmd
    can_.frame.data[5] = 0;
    can_.frame.data[6] = 0;
    can_.frame.data[7] = 0;

//    printf("\t\tID : %0.3x \t DLC : %d \t DATA : ", tmp_id, tmp_dlc);
//    for(int i = 0; i < 8; i++)
//        printf("%0.2x ", can_.frame.data[i]);
//    printf("\n");
    
    can_.can_write();
}


bool Report_APM(CAN_AVL &can_)      // id = 0x710
{
    bool can_error_flag_ = 0;

    //    can_print(can_);

    bool APM_Fd_En_         = 0;        // APM Manual/Auto 모드 변환 요청 피드백
    bool APM_Fd_Override_    = 0;        // APM override check
    int16_t APM_Fd_SteerAngle = 0;

    //    u_int16_t abc = can_.frame.data[3];
    //    u_int16_t abc_2 = abc << 8;

    APM_Fd_En_ = can_.frame.data[0] & 0x1;
    APM_Fd_Override_ = can_.frame.data[1] & 0x1;
    APM_Fd_SteerAngle = can_.frame.data[3]& 0xffff;
    APM_Fd_SteerAngle = APM_Fd_SteerAngle << 8;
    APM_Fd_SteerAngle = can_.frame.data[2]|APM_Fd_SteerAngle;
    APM_Fd_SteerAngle *= 0.1;
    //    std::cout<<std::dec<< "핸들 각도 : "<< 0.1*APM_Fd_SteerAngle <<std::endl; // -483 ~ + 483 (+-485 까지 가능하나, 너무 부담됨)
    vehicle_steering_angle = APM_Fd_SteerAngle;

    if(print_can_control_state_flag == 1)
    {
        if(APM_Fd_En_ == true)      printf("APM Enable flag\n");
        else                        printf("APM Disable flag\n");
    }

    if(APM_Fd_Override_ == true) printf("APM Override flag\n");

    if(print_steer_angle_flag == 1) printf(" APM_Fd_SteerAngle : %d\n",APM_Fd_SteerAngle);

    return can_error_flag_;
}


bool Report_ASM(CAN_AVL &can_)      // id = 0x711
{
    bool can_error_flag_ = 0;

    //    can_print(can_);

    bool ASM_Fd_En_         = 0;        // ASM Manual/Auto 모드 변환 요청 피드백
    u_int8_t ASM_Fd_VSpeed_  = 0;        // ASM vehicle speed output
    u_int8_t Mo_Fd_AlvCnt_   = 0;        // Alive-count feedback value

    ASM_Fd_En_          = can_.frame.data[0] & 0x1;
    ASM_Fd_VSpeed_       = can_.frame.data[1] & 0xff;
    Mo_Fd_AlvCnt_        = can_.frame.data[7] & 0xff;


    //printf("ASM_Fd_En : %d\n", ASM_Fd_En_);
    if(print_can_control_state_flag == 1)
    {
        if(ASM_Fd_En_ == true)      printf("ASM Enable flag\n");
        else                        printf("ASM Disable flag\n");
    }

    if(print_vehicle_speed_flag == 1)    printf("Vehicle Speed \t\t: %d\n", ASM_Fd_VSpeed_);
    if(print_alive_count_flag == 1)    printf("Alive-count value \t\t: %d\n", Mo_Fd_AlvCnt_);

    //    if(speed_pid_control_enable_flag == 1)

    current_vehicle_speed = ASM_Fd_VSpeed_;
    if(print_cluster_speed_display_flag == 1) cluster_speed_display_value = ASM_Fd_VSpeed_;
    //printf("APM_Fd_En_ \t= %d \t\t APM_Fd_Override \t=%d\n", APM_Fd_En_, APM_Fd_Override);

    return can_error_flag_;
}



bool Report_Misc(CAN_AVL &can_)
{
    bool can_error_flag_ = 0;
    // Repotr Misc


    /*
     코드 작성하기
     */

    return can_error_flag_;
}

void TCU_DVT13_Driving_Tq(CAN_AVL can_)
{
    u_int16_t driving_tq = 0;

    driving_tq = can_.frame.data[0] | ((u_int16_t)(can_.frame.data[1] << 6) << 2);

    if(print_driving_tq_flag == 1) printf("Driving_Tq : %d\n", driving_tq);
}

void TCU_DCT13_Engine_RPM(CAN_AVL can_)
{
    //    can_print(can_);

    // (0.9766, 0)
    u_int16_t cluster_engine_RPM = 0;
    bool cluster_engine_RPM_flag = 0;

    cluster_engine_RPM = (((u_int16_t)(can_.frame.data[2] << 1)) << 5) |(can_.frame.data[1] >> 2);
    cluster_engine_RPM_flag = (can_.frame.data[2] >> 7);

    cluster_engine_RPM = cluster_engine_RPM * 1;    // real value mul

    if(print_cluster_engine_rpm_flag == 1) printf("Cluster Engine RPM : %d\n", cluster_engine_RPM);
}

void NEW_MSG_ENGINE(CAN_AVL can_)
{
    u_int16_t rpm_1 = 0;
    u_int16_t rpm_2 = 0;

    rpm_1 = can_.frame.data[0] | ((u_int16_t)can_.frame.data[1] & 0x0f) << 8; // 0b0000111 == 0x0f
    rpm_2 = can_.frame.data[2] | ((u_int16_t)can_.frame.data[3]) << 8;

    if(print_new_msg_engine_rpm_flag == 1)
    {
        printf("NEW_MSG_ENGINE RPM_1 : %d\n", rpm_1);
        printf("NEW_MSG_ENGINE RPM_2 : %d\n", rpm_2);
    }

}

void NEW_MSG_2_RPM_WITHOUT_ELECTRO(CAN_AVL can_)
{
    // bit범위가 Brake_Pressure랑 겹침.
    //  1. dbc 잘못 읽거나
    //  2. dbc 잘못 쓰여졌거나
    u_int16_t rpm_without_electro = 0;

    rpm_without_electro = can_.frame.data[2] | ((u_int16_t)can_.frame.data[3] << 8);

    if(print_rpm_without_electro_flag == 1) printf("RPM_Without_Electro : %d\n", rpm_without_electro);
}

void Brake_Pressure(CAN_AVL can_)
{
    /*
     주석된 부분은 기존에 정상작동으로 보이던 코드.
     dbc 기준 Motorola, Intel의 코드 해석법이 다르다.

     */

    //        u_int16_t brake_pressure = 0;
    //
    //        brake_pressure = (can_.frame.data[0] >> 7) | (((u_int16_t)can_.frame.data[1]) << 1) | (((u_int16_t)can_.frame.data[2]) << 9);
    //
    //        brake_pressure = brake_pressure * 1;    // real value mul

    u_int16_t brake_pressure = 0;

    // 7|16@0+ -> big-endian 방식으로 7bit부터 오른쪽 방향으로 16bit 읽음
    brake_pressure = can_.frame.data[1] | (((u_int16_t)can_.frame.data[0]) << 8);

    brake_pressure = brake_pressure * 1;    // real value mul

    if(print_brake_pressure_flag == 1) printf("Brake_Pressure : %d\n", brake_pressure);
}

void WHL_Speed(CAN_AVL can_)
{
    u_int16_t whl_spd_rl = 0;   //wheel speed rear left
    u_int16_t whl_spd_rr = 0;   //wheel speed rear right

    whl_spd_rl = (can_.frame.data[4]) | ((((u_int16_t)can_.frame.data[5]) & 0x3f) << 8);    // 0b00111111 == 0x3f
    whl_spd_rr = (can_.frame.data[6]) | ((((u_int16_t)can_.frame.data[7]) & 0x3f) << 8);

    if(print_whl_spd_flag == 1)
    {
        printf("whl_spd_rl : %d\n", whl_spd_rl);
        printf("whl_spd_rr : %d\n", whl_spd_rr);
    }

}

void WHL_PUL(CAN_AVL can_)
{
    u_int8_t whl_pul_rl = 0;
    u_int8_t whl_pul_rr = 0;

    whl_pul_rl = can_.frame.data[2];
    whl_pul_rr = can_.frame.data[3];

    if(print_whl_pul_flag == 1)
    {
        printf("whl_pul_rl : %d\n", whl_pul_rl);
        printf("whl_pul_rr : %d\n", whl_pul_rr);
    }

}

void YAW_RATE(CAN_AVL can_)
{
    uint16_t Yaw_rate = 0;

//    Yaw_rate = can_.frame.data[6] & 0x1f;
//    Yaw_rate = Yaw_rate << 8;
//    Yaw_rate = (int16_t)can_.frame.data[5] | Yaw_rate;

    Yaw_rate = (uint16_t)(can_.frame.data[5] & 0xff) | ((uint16_t)(can_.frame.data[6]&0x1f)<<8);
    
    vehicle_yaw_rate = 0.01 * Yaw_rate - 40.20; // 40.95는 dbc에 쓰여진 offset

    // Yaw Rate가 dbc상 오프셋값에 더하여 바이어스 애러가 있으므로 누적값으로 애러를 잡아주는 것. while문 2000번 반복횟수까지 누적.
    if(yaw_rate_count < 1000)
    {
        yaw_rate_count++;
        yaw_rate_accum_value += vehicle_yaw_rate;
        yaw_rate_error_mean_value = yaw_rate_accum_value / yaw_rate_count;
        
        printf("Yaw Rate Calibration ... Waitting for minutes!, mean error : %f \t count : %d\n", yaw_rate_error_mean_value, yaw_rate_count);
        printf("Yaw Rate Calibration ... Waitting for minutes!, mean error : %f \t count : %d\n", yaw_rate_error_mean_value, yaw_rate_count);
        printf("Yaw Rate Calibration ... Waitting for minutes!, mean error : %f \t count : %d\n", yaw_rate_error_mean_value, yaw_rate_count);
        printf("Yaw Rate Calibration ... Waitting for minutes!, mean error : %f \t count : %d\n", yaw_rate_error_mean_value, yaw_rate_count);
        printf("Yaw Rate Calibration ... Waitting for minutes!, mean error : %f \t count : %d\n", yaw_rate_error_mean_value, yaw_rate_count);
    }
    else
    {
        // 차량에서 나온 yaw_rate에 누적된 초기오차를 빼준 것.
        vehicle_yaw_rate +=-yaw_rate_error_mean_value;
        vehicle_yaw_rate_error_correct = 1;
//        if(print_yaw_rate_flag == 1)
//            printf("Yaw Rate Calibration is complete!!!!, yaw rate : %f\n", vehicle_yaw_rate);
    }
    
    if(print_yaw_rate_flag == 1) printf("vehicle_yaw_rate : %f\n", vehicle_yaw_rate);


}

void can_print(CAN_AVL can_)
{
    printf("  - R | HEX | ID:%04x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
           (int) can_.frame.can_id, (int) can_.frame.can_dlc, (int) can_.frame.data[0],
           (int) can_.frame.data[1], (int) can_.frame.data[2],
           (int) can_.frame.data[3], (int) can_.frame.data[4],
           (int) can_.frame.data[5], (int) can_.frame.data[6],
           (int) can_.frame.data[7]);
}
