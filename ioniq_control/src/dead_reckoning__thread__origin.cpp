
/*
     GNSS 의 느린 Hz를 보정해줄 DR 구현 스레드
 */

#include <stdio.h>
#include <iostream>
#include "env_setting.h"
#include <sys/time.h>
#include <unistd.h>
#include <queue>

// Read Coordinates Map Data
//void Coordinates_Map_Data();

void* Dead_Reckoning(void *DR_error_flag_)
{
    double yaw_rate_accum_tmp = 0;
    
    // 스티어링 각도의 변화량을 계산하기 위한 값.
    std::queue<double> steering_angle;
    steering_angle.push(0.0);

    // check running time
    double elapsedTime = 0;

    while(1)
    {
        if(gnss_initial_flag == 1)
        {
            // check running time
            struct timeval t1, t2;
//            double elapsedTime;
            
            // start timer
            gettimeofday(&t1, NULL);
            
//            usleep(100);
            
            
            
            // Dead reckoning
            // 시간 기준은 GNSS thread의 elapsedTime으로 잡음. 해당 갱신이 chassis CAN의 갱신보다 빠르면 약간의 오차가 발생함. 나중에 mutex로 잡으면 될 듯.
            
            // Dead Reckoning으로 yaw_angle 구하기
            //  rad/s?? degree/s?? * ms
            //  우선 deg/s로 계산 --> 환산하면 1/1000 deg / ms
            //  얘 단위 수정해야함. **********************************************

            if(vehicle_yaw_rate_error_correct == 1)
                dr_yaw_rate_integral_val = -vehicle_yaw_rate / 1000 * elapsedTime;
            
            
            //            printf("elapsedTime = %f\n", elapsedTime);
            //            printf("vehicle_yaw_rate = %f\n", vehicle_yaw_rate);
            //            printf("dr_yaw_rate_integral_val = %f\n", dr_yaw_rate_integral_val);
            
            //            printf("steering angle = %f\n", vehicle_steering_angle);    // 1도당 0.0819도
            
            //            printf("gnss_yaw_angle = %f\n", gnss_yaw_angle);
            
            
            // steering angle 기반 dr yaw angle (재대로 계산)
//            steering_angle.push(vehicle_steering_angle * 0.06);//0.09);
//            double steering_angle_tmp = steering_angle.front();
////            std::cout << "steering_angle_tmp = " << steering_angle_tmp << std::endl;
//            steering_angle.pop();
//            steering_angle_tmp += -steering_angle.front();
//            dr_yaw_rate_integral_val = steering_angle_tmp;// / 1000 * elapsedTime;
            
            
            // 기존 yaw_angle에 적분된 yaw_rate (=yaw_angle) 더하기
//            gnss_yaw_angle += dr_yaw_rate_integral_val;                 // gnss_yaw_angle += 적분 vehicle_yaw_rate(chassis CAN);
//            dr_yaw_angle = gnss_yaw_angle;// + dr_yaw_rate_integral_val;
            
            // steering angle기반 dr yaw angle (야매)
//            dr_yaw_angle = gnss_yaw_angle - vehicle_steering_angle * 0.09;
            
            
            // 앞 - 뒤 (부호)
            
            // 스티어앵글 변화량
//            double steering_rate = vehicle_steering_angle * 0.09;
            
            // 이전 스티어앵글 각 - 지금 스티어앵글 값 에다 시간 계산해서 요레이트 뽑기?
            
            // yaw rate 누적에 대한 결과를 따로 빼서 계산. 얘가맞음!
            yaw_rate_accum_tmp += dr_yaw_rate_integral_val;
            if(vehicle_yaw_rate_error_correct == 1)
                gnss_yaw_angle = gnss_yaw_angle + dr_yaw_rate_integral_val;
            dr_yaw_angle = gnss_yaw_angle;
            
//            std::cout << "dr_yaw_angle = " << dr_yaw_angle << "\t\tyar_rate_accum_tmp = " << yaw_rate_accum_tmp << "\t\tdr_yaw_rate_integral_val = "<< dr_yaw_rate_integral_val << "\t\tvehicle_yaw_rate = "<< vehicle_yaw_rate << std::endl;
            
//            printf("dr_yaw_angle = %f\n", dr_yaw_angle);
            
            // x축 변화량 구하기 (elapsedTime을 이용해 적분 효과)
            // current_vehicle_speed가 kph이므로 m/ms로 바꾸려면 3600을 나눠주면 됨.
            delta_dr_x = elapsedTime * (double)current_vehicle_speed / 3600. * cos(3.1415926 / 180. * dr_yaw_angle);
//            delta_dr_x = elapsedTime * (double)current_vehicle_speed / 3600. * cos(3.1415926 / 180. * (gnss_yaw_angle - vehicle_steering_angle * 0.09));
            
//            printf("current_vehicle_speed = %d\n", current_vehicle_speed);
//            printf("yaw_rate = %f\n", vehicle_yaw_rate);
            
            //            printf("elapsedTime * current_vehicle_speed / 3600 = %f\n", elapsedTime * (double)current_vehicle_speed / 3600.);
            //            printf("cos(3.1415926 / 180 * dr_yaw_angle) = %f\n", cos(3.1415926 / 180. * dr_yaw_angle));
            
            // 기존 x좌표에 x축 변화량 더하기
            gnss_x += delta_dr_x;
            dr_x = gnss_x;// + delta_dr_x;
            
            //            dr_hard_x += delta_dr_x;
            //            dr_x = dr_hard_x;
            
            
            // y축 변화량 구하기 (elapsedTime을 이용해 적분 효과)
            // current_vehicle_speed가 kph이므로 m/ms로 바꾸려면 3600을 나눠주면 됨.
            delta_dr_y = elapsedTime * (double)current_vehicle_speed / 3600. * sin(3.1415926 / 180. * dr_yaw_angle);//cos(3.1415926 / 180. * (90. - dr_yaw_angle));
//            delta_dr_y = elapsedTime * (double)current_vehicle_speed / 3600. * sin(3.1415926 / 180. * (gnss_yaw_angle - vehicle_steering_angle * 0.09));
            //cos(3.1415926 / 180. * (90. - (gnss_yaw_angle - vehicle_steering_angle * 0.0819)));
            
            // 기존 y좌표에 y축 변화량 더하기
            gnss_y += delta_dr_y;
            dr_y = gnss_y;// + delta_dr_y;
            
            //            dr_hard_y += delta_dr_y;
            //            dr_y = dr_hard_y;
            
            
            // 최종 출력 : dr_yaw_angle, dr_x, dr_y
            //            std::cout << std::fixed;
            //            std::cout.precision(7);
            //            std::cout<< dr_x << "\t\t" << dr_y << "\t\t" << dr_yaw_angle << "\t\t" << "DR" << std::endl;
            
#ifdef OpenCV_View_MAP
            
            //            pthread_mutex_lock(&mutex); // 잠금을 생성한다.
            // opencv viewer에 그릴 좌표 설정
            opencv_x = dr_x * 4;
            opencv_y = dr_y * 4;
            //            printf("elapsedTime = %f \t\t delta_dr_x = %f \t\t opencv_x = %f \t\t opencv_y = %f\n", elapsedTime, delta_dr_x, opencv_x, opencv_y);
            //            pthread_mutex_unlock(&mutex); // 잠금을 생성한다.
            
            // cv::circle을 이용한 좌표 그리기
            cv::Point dot(dr_x*4, dr_y*4);
            cv::circle(img, dot, 1, CvScalar(0,255,255));
            
            // img data에 접근하여 픽셀 변경을 통한 좌표 그리기
            //            img.data[(int)(dr_y*4) * img.cols * 3 + (int)(dr_x*4) * 3 + 0] = 0;
            //            img.data[(int)(dr_y*4) * img.cols * 3 + (int)(dr_x*4) * 3 + 1] = 255;
            //            img.data[(int)(dr_y*4) * img.cols * 3 + (int)(dr_x*4) * 3 + 2] = 255;
            
            //            usleep(1);
            //            count_dr++;
            //            printf("count_dr = %d\n", count_dr);
            // imshow는 main()에서. thread에서는 imshow 애러뜸.
#endif
            
            // finish timer
            gettimeofday(&t2, NULL);
            
            // compute and print the elapsed time in millisec
            elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
            elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
//            std::cout << std::fixed;
//            std::cout.precision(4);
//            std::cout << elapsedTime << " ms.\n";
        }
    }
    
    
}
