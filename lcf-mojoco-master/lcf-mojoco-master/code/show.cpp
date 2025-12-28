#include "zf_common_headfile.h"


void show_all()
{
    // ips200_show_gray_image(0, 0, (uint8_t*)img5, 160, 120);
    // ips200_show_uint(0,8*24,Threshold_real,3);
    // ips200_show_gray_image(0, 0, (uint8_t*)img3, 80, 60);

    show_two();  //二值化

    // ips200_show_gray_image(0, 0, (uint8_t*)img_temp, 80, 60);
    
    ips200_show_float(90,2,stree_erro,3,2);
    ips200_show_int(100,2,stree_out,3);
    ips200_show_int(0,8*18,encoder_left,3);
    ips200_show_int(0,8*20,encoder_right,3);
    
}