//
//  pd_control.h
//  pcan_mac
//
//  Created by Changhyeon Park on 16/08/2018.
//

#ifndef pd_control_h
#define pd_control_h

class PD_CONTROL
{
public:
    float target_value ;
    float real_value;
    
    float e_n;
    float e_b;
    
    float MV;
    
    float K_p;
    float K_d;
    
    PD_CONTROL();
    float MV_Cal_Func(int target_val_, int real_val_, float K_p_, float K_d_);
};

PD_CONTROL::PD_CONTROL()
{
    target_value = 0;
    real_value = 0;
    e_n = 1;
    e_b = 0;
    MV = 0;
    K_p = 0;
    K_d = 0;
}

float PD_CONTROL::MV_Cal_Func(int target_val_,  int real_val_, float K_p_, float K_d_)
{
    target_value = target_val_;
    real_value = real_val_;
    K_p = K_p_;
    K_d = K_d_;
//    printf("target_val_ : %d \t real_value : %d\n", target_value, real_value);
    e_n = target_value - real_value;
//    printf("e_n : %f\n", e_n);
    float MV_tmp = 0;
    MV_tmp = K_p * e_n + K_d * (e_n - e_b);
    MV_tmp = MV_tmp < 0 ? MV_tmp * 3.5 : MV_tmp;
    
//    printf("MV = %f\n", MV_tmp);
    e_b = e_n;
    return MV_tmp;
//    return 0;
}

#endif /* pd_control_h */
