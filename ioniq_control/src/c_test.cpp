#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <fstream>
#include <string.h>

//int pthread_mutex_unlock(pthread_mutex_t *mutex);
//int pthread_mutex_lock(pthread_mutex_t *mutex);
//int pthread_mutex_trylock(pthread_mutex_t *mutex);
//int pthread_mutex_destory(pthread_mutex_t *mutex);

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

int ncount;    // 쓰레드간 공유되는 자원
pthread_mutex_t  mutex = PTHREAD_MUTEX_INITIALIZER; // 쓰레드 초기화
int tmp__ = 3;

void* do_loop(void *data)
{
    int i;
    for (i = 0; i < 20; i++)
    {
        tmp__ = ncount;
        pthread_mutex_lock(&mutex); // 잠금을 생성한다.
        ncount ++;
        printf("loop1 : %d\n", ncount);
        //while(1);
        //if(i == 20) return;
        pthread_mutex_unlock(&mutex); // 잠금을 해제한다.
        sleep(1);
    }
}

void* do_loop2(void *data)
{
    int i;
    
    // 잠금을 얻으려고 하지만 do_loop 에서 이미 잠금을
    // 얻었음으로 잠금이 해제될때까지 기다린다.
    for (i = 0; i < 20; i++)
    {
        int tmp_ = 0;
        
        if(16 == pthread_mutex_trylock(&mutex)) // 잠금을 생성한다.
        {
            ncount = tmp__;
            tmp_ = 16;
        }
        else tmp_ = 0;
        ncount ++;
        printf("loop2 : %d, trylock = %d\n", ncount, tmp_);
        pthread_mutex_unlock(&mutex); // 잠금을 해제한다.
        sleep(1);
    }
}

#include "c_test.h"

__test_ _test___;

void test_();
extern int test_val;

void *test_thread(void *data);
#include "math.h"
#include <queue>
typedef struct TSR
{
    int TSR_class[4];
    int64 TSR_xmin[4];
    int TSR_ymin[4];
    int TSR_xmax[4];
    int TSR_ymax[4];
} TSR;

#define RECEIVE_FREQ(hz) 5/hz



#define HEADING_MOVING_AVG 7000 //count

#ifdef HEADING_MOVING_AVG
std::vector<double> heading_Mavg_storage;
int heading_Mavg_count = 7000;
#endif


template <typename T>
T Mavg(T input_, std::vector<T>& Mavg_storage_tmp_, int count);

double find_small_value(double value_[], int size_)
{
    double tmp = value_[0];
    for(int i = 1; i < size_; i++)
    {
        // 작은거 찾기
        tmp = value_[i] > tmp ? tmp : value_[i];
    }
    
    return tmp;
}

int main()
{
    double *list_test;
    list_test = (double *)malloc(sizeof(double));
    list_test[0] = 30;
    list_test[1] = 3;
    list_test[2] = 32;
    list_test[3] = 9;
    list_test[4] = 11;
    
    double small_ = find_small_value(list_test, 5);
    std::cout << small_ << std::endl;

    double input_ = 1;
    double output_ = 0;
    
    for(int i = 0; i < 10000; i++)
    {
        output_ = Mavg<double>(i, heading_Mavg_storage, heading_Mavg_count);
        //std::cout << output_ << std::endl;
    }
    
    std::cout << atan2(-3, 0) * 180 / 3.141592 << std::endl;
    std::cout << tan(10 * 3.141592 / 180) << std::endl;
    int fsdivsdvzs = 3;
    
#ifdef RECEIVE_FREQ
    std::cout << RECEIVE_FREQ(1) << std::endl;
#endif
    
    double abc;
    abc = sqrt(0);
    std::cout << abc << std::endl;
    abc = sqrt(-0.1);
    std::cout << abc << std::endl;
    
    double aaa = 1./0.;
    std::cout << aaa << std::endl;
    std::cout << atan(aaa) * 180 / 3.141592 << std::endl;
    std::queue<double> q_test;
    q_test.push(0);
    q_test.push(1);
    q_test.push(2);
    q_test.push(3);
    q_test.push(4);
    
    std::cout << q_test.front() << std::endl;
    
    int aaaaa = 10;
    int bbbbb = 2;
    
    aaaaa /= bbbbb;
    std::cout << aaaaa << std::endl;
    
    TSR *tsr;
    tsr = (TSR*)malloc(sizeof(TSR));
    
    tsr[0].TSR_class[0] = 1;
    //memset(tsr, 1, sizeof(TSR));
    std::cout << tsr[0].TSR_class[0] << tsr[0].TSR_xmin[0] << tsr[0].TSR_xmax[0] << std::endl;
    
    free(tsr);
//    tsr.TSR_class[0] = 0;
//    tsr.TSR_xmin[0] = 0;
//    tsr.TSR_ymin[0] = 0;
//    tsr.TSR_xmax[0] = 10;
//    tsr.TSR_ymax[0] = 10;
    
    
    
    
    std::string *Class;
    Class = (std::string *)malloc(sizeof(std::string));
    Class[0] = "what";
    std::cout << Class[0] << std::endl;
    
#ifdef __APPLE__
    std::cout << "test tt" << std::endl;
    std::ofstream fin2;
    fin2.open("../../test_c_test.txt");
    fin2 << "test test test " << std::endl;
    fin2.close();
#endif
    

    
    // y, x
    std::cout << atan2(10, 0.000001) * 180 / 3.1415926 << std::endl;
    std::cout << sin(3.1415926 / 180. * 30) << std::endl; 

    pthread_t thread[3];
    
    pthread_create(&thread[0], NULL, test_thread, NULL);
    
    while(1)
    {
        printf("t\n");
    }

    pthread_join(thread[0], NULL);
    
    _test___.a = 3;
    test_val = 37;
    test_();
    std::cout << _test___.b << std::endl;
//    cv::Mat test(300, 300, CV_8UC3);

//    test = cv::imread("/Users/CHP/Desktop/test.png");
//    while(1)
//    {
//        test_();
//        cv::namedWindow("test");
//        cv::imshow("test", test);
//        if(cv::waitKey(10) == 'q') {break;}
//    }

    struct timeval t1, t2;
    double elapsedTime;
    
    // start timer
    gettimeofday(&t1, NULL);

    // do something
    // ...
    cv::waitKey(1000);  // ms
    std::cout << cos(3.14 / 180 * 60) << std::endl << std::endl;
    // stop timer
    gettimeofday(&t2, NULL);
    
    // compute and print the elapsed time in millisec
    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
    std::cout << elapsedTime << " ms.\n";
    
    cv::Mat img(700, 300, CV_8UC1);
    img.setTo(0);
    
    std::ifstream fin;
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/gnss_receive_data_Hanyang_1_copy.txt");
    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/gnss_receive_data_전체.txt");
    double latitude, longitude, heading;
    while(fin>>latitude>>longitude>>heading)
    {
        // Starting point   : 36.72790320        127.44262740
        // longitude        : 111km per 1degree.                    = 1.1cm per 0.0000001degree
        // latitude         : 88.8km per 1degree at longitude 37    = 0.88cm per 0.0000001degree
        std::cout << std::fixed;
        std::cout.precision(7);
        
        // How to change Wold Coordinate to Relative Coordinate
        // latitude(cm) = (latitude(degree) - Starting point(degree)) * 10,000,000 * 0.888
        // longitude(cm) = (longitude(degree) - Starting point(degree)) * 10,000,000 * 1.1
        
        // calculated as m (+ 30)
        double x =(latitude - 36.72790320)* 10000000 * 0.888 / 100. + 30;
        double y =(longitude - 127.44262740) * 10000000 * 1.1 / 100. + 10;
        
        std::cout << x <<"\t\t"<< y << "\t\t" <<heading << std::endl;
        img.data[(int)(y*4) * img.cols + (int)(x*4)] = 255;
    }
    std::cout << "finished" << std::endl;
    
    fin.close();
    while(1)
    {
        cv::imshow("Coordinate", img);
        if(cv::waitKey(10) == 'q')  break;
    }
    
    
//    std::ofstream fin;
////    std::ofstream fout;
//    fin.open("/Users/CHP/ioniq/Code_CHP/PEAKCAN_Mac/c++/gnss_receive_data.txt");
//    double aa[4] = {0, 1.121232, 2.232132};
//    for(int i = 0; i < 10; i++)
//    {
//        fin<<aa[0]<<"\t\t";
//        fin<<aa[1]<<"\t\t";
//        fin<<aa[2]<<std::endl;
//        printf("test\n");
//    }
    
    fin.close();
    
    int       thr_id;
    pthread_t p_thread[2];
    int status;
    int a = 1;
    
    ncount = 0;
    thr_id = pthread_create(&p_thread[0], NULL, do_loop, (void *)&a);
    sleep(1);
    thr_id = pthread_create(&p_thread[1], NULL, do_loop2, (void *)&a);
    
    pthread_join(p_thread[0], (void **) &status);
    pthread_join(p_thread[1], (void **) &status);
    
    status = pthread_mutex_destroy(&mutex);
    printf("code  =  %d", status);
    printf("programing is end");
    return 0;
}

////
////  c_test.cpp
////  pcan_mac
////
////  Created by Macbook Pro on 13/08/2018.
////
//
//#include <stdio.h>
//#include <iostream>
//#include <pthread.h>
//#include "sys/time.h"
//#include <stdio.h>
//#include <unistd.h>
//#include <stdlib.h>
//
//
//void sys_time_header_example();
//float Vehicle_Speed();
//
////void* do_loop(void *data)
////{
////    int i;
////
////    int me = *((int *)data);
////    for (i = 0; i < 10; i++)
////    {
////        printf("%d - Got %d\n", me, i);
////        sleep(1);
////    }
////}
////
////int main()
////{
////    int       thr_id;
////    pthread_t p_thread[3];
////
////    int status;
////    int a = 1;
////    int b = 2;
////    int c = 3;
////
////    thr_id = pthread_create(&p_thread[0], NULL, do_loop, (void *)&a);
////    thr_id = pthread_create(&p_thread[1], NULL, do_loop, (void *)&b);
////    thr_id = pthread_create(&p_thread[2], NULL, do_loop, (void *)&c);
////
////    while(1)
////    {
////        printf("test\n");
////        sleep(1);
////    }
////
////    pthread_join(p_thread[0], (void **) &status);
////    pthread_join(p_thread[1], (void **) &status);
////    pthread_join(p_thread[2], (void **) &status);
////
////    printf("programing is end\n");
////    return 0;
////}
//
//u_int16_t data_float;
//
//void* testt(int a_)
//{
//    a_ = 3;
//    return (void*)a_;
//}
//
//int main()
//{
//    int test;
//    void* abcd = testt(test);
//    test = *(int*)abcd;
//    printf("%d\n", test);
//
//    struct timeval t1, t2;
//    double elapsedTime;
//
//    // start timer
//    gettimeofday(&t1, NULL);
//
//    // do something
//    // ...
//
//    for(int i = 0; i < 100; i++)
//        for(int j = 0; j < 100; j++)
//            for(int c = 0; c < 100; c++)
//            {
//                int a = 3;
//            }
//
//    // stop timer
//    gettimeofday(&t2, NULL);
//
//    // compute and print the elapsed time in millisec
//    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
//    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
//    std::cout << elapsedTime << " ms.\n";
//
//    /*
//
//     input_speed = whl_spd_rl $$$ whl_sped_rr;
//     or
//     input_speed = can_output_vehicle_speed;
//
//     */
//
//    u_int16_t target_speed = 10;
//    u_int16_t input_speed = 0;
//    u_int16_t output_speed = 0;
//
//    // 현재 input speed 갱신 (renew VSpeed from CAN_Read)
//    input_speed = Vehicle_Speed();
//
//    float error = target_speed - input_speed;   // or use MSE
//
//    float time = 0.;
//
//    float K_p = 0.5;
//    float K_d = 0;
//
//    // P control    // K_p * error
//
//    // D control    // gradient
//
//
//
//
//
//
//
//
//
////    u_int8_t a = 0b11111111;
////    u_int8_t b = 0b11111111;
////
////    u_int16_t whl_spd_rr = a | ((((u_int16_t)b) & 0x7f) << 8);
////
////    printf("%04x\n", whl_spd_rr);
////
////
////    u_int16_t test = (a) | ((((u_int16_t)b) & 0x3f) << 8);
////    printf("%04x\n", test);
//
//    return 0;
//}
//
//float Vehicle_Speed()
//{
//    float tmp = 0;
//
//    // do something
//    // ...
//
//    return tmp;
//}
//
//void sys_time_header_example()
//{
//    struct timeval t1, t2;
//    double elapsedTime;
//
//    // start timer
//    gettimeofday(&t1, NULL);
//
//    // do something
//    // ...
//
//    // stop timer
//    gettimeofday(&t2, NULL);
//
//    // compute and print the elapsed time in millisec
//    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
//    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
//    std::cout << elapsedTime << " ms.\n";
//}

template <typename T>
T Mavg(T input_, std::vector<T>& Mavg_storage_tmp_, int count)
{
    T input_tmp_ = input_;
    T output_tmp_ = 0;
    
    Mavg_storage_tmp_.push_back(input_tmp_);
    std::cout << "size applied push : " << Mavg_storage_tmp_.size() << std::endl;
    
    if(Mavg_storage_tmp_.size() > count) Mavg_storage_tmp_.erase(Mavg_storage_tmp_.begin());//Mavg_storage_tmp_.pop_back();
    std::cout << "size applied pop : " << Mavg_storage_tmp_.size() << std::endl;
    
    for(int i = 0; i < Mavg_storage_tmp_.size(); i++)
    {
        output_tmp_ += (Mavg_storage_tmp_[i] / count);
        //std::cout << Mavg_storage_tmp_[i] << std::endl;
    }
    
    return output_tmp_;
}
