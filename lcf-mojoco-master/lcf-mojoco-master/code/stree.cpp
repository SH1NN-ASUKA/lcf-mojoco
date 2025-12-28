
#include "zf_common_headfile.h"
// struct pwm_info pwm_1_info;
// struct pwm_info pwm_2_info;
struct pwm_info servo_pwm_info;

// #define PWM_1           "/dev/zf_device_pwm_esc_1"
// #define PWM_2           "/dev/zf_device_pwm_esc_2"
#define SERVO_MOTOR1_PWM            "/dev/zf_device_pwm_servo"
#define PWM_DUTY_MAX                (servo_pwm_info.duty_max) 
float stree_erro,stree_last_erro;
float stree_KP=0,stree_KD=0;
int stree_pid_out;
int32_t stree_out;
int stree_BeiShu;
int stree_BeiShu_wd;
int stree_BeiShu_zd;
int32 stree_pid_out_zhidao;

//直道PID参数
float ZD_P = 4;//0.6  1  0.8
float ZD_D = 5;//1    1.4   0.7

//左圆环PID参数
float ZY_P = 8;
float ZY_D = 15;

//右圆环PID参数
float YY_P = 8;
float YY_D = 15;

//弯道PID参数
float WD_P=10;
float WD_D=25;//3.8//3.2


//舵机初始化
void Stree_Init(void)
{
    // 获取PWM设备信息
    // pwm_get_dev_info(PWM_1, &pwm_1_info);
    // pwm_get_dev_info(PWM_2, &pwm_2_info);
    pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
    // printf("servo pwm freq = %d Hz\r\n", servo_pwm_info.freq);
    // printf("T:%d \r\n", PWM_DUTY_MAX);
}


void Stree_Duty(uint32_t duty)
{
    
    if(duty>stree_left)
        duty=stree_left;
    if(duty<stree_right)
        duty=stree_right;
    pwm_set_duty(SERVO_MOTOR1_PWM, duty);    
}



MH      MHstruct;
MH      MHstructFastCar = {

        {2.4, 2.55, 2.65, 2.9, 3.1,3.4,3.7},//2.3, 2.45, 2.65, 2.9, 3.1,3.4,3.7//2.2, 2.4, 2.6, 2.9, 3.1,3.4,3.7// 3.3, 3.6, 3.9, 4.45, 5,5.45,5.95  //76
        {2.4, 2.55, 2.65, 2.9, 3.1,3.4,3.7},
   {
    {
      {//L-IN
              { 0, 1, 1, 1, },
              { 1, 2, 2, 2, },
              { 2, 3, 3, 3, },
              { 3, 4, 4, 4, }
      }
    },

    {
      {//R-IN
              { 0, 1, 1, 1, },
              { 1, 2, 2, 2, },
              { 2, 3, 3, 3, },
              { 3, 4, 4, 4, }
      }
    },

    {
      {//L-OUT
              { 0, 1, 1, 1, },
              { 1, 2, 2, 2, },
              { 2, 3, 3, 3, },
              { 3, 4, 4, 4, }
      }
    },

    {
      {//R-OUT   
              { 0, 1, 1, 1, },
              { 1, 2, 2, 2, },
              { 2, 3, 3, 3, },
              { 3, 4, 4, 4, }
      }
    }
   }
};



void DuoJi_GetP(float *i32p_P, int16_t i16_ViewH, int16_t i16_ViewE)
{

  MHstruct.i16_ViewH = i16_ViewH;

  float VH = f_Get_H_approximation(i16_ViewH - H_Min);
  float VE = f_Get_E_approximation(i16_ViewE, MHstruct.f_SizeOfViewE);
  float X2Y = 0;
  float X1Y = 0;
  float Y2X = 0;
  float Y1X = 0;

  int8_t VH1 = (int)VH;
  if (VH1 > VH) {
    VH--;
  }
  int8_t VH2 = VH1 + 1;

  int8_t VE1 = (int)VE;
  if (VE1 > VE) {
    VE1--;
  }
  int8_t VE2 = VE1 + 1;

  if (VH1 > 2) {
    VH1 = 2;
  }

  if (VH2 > 2) {
    VH2 = 2;
  }

  if (VE1 > 2) {
    VE1 = 2;
  }

  if (VE2 > 2) {
    VE2 = 2;
  }

  X2Y = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1]) *
            (VE - VE1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1];

  X1Y = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1]) *
            (VE - VE1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1];

  Y2X = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1]) *
            (VH - VH1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1];

  Y1X = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2]) *
            (VH - VH1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2];

  float P_approximation = (X2Y + X1Y + Y2X + Y1X) / 8.0;

  int8_t P1 = (int)P_approximation;
  if (P1 > P_approximation) {
    P1--;
  }
  int8_t P2 = P1 + 6;

    if (i16_ViewE < 0) {

  *i32p_P = (MHstruct.f_DuoJiP_TableL[P2] - MHstruct.f_DuoJiP_TableL[P1])*(P_approximation - P1) +MHstruct.f_DuoJiP_TableL[P1];
    }
    else
    {
  *i32p_P = (MHstruct.f_DuoJiP_TableR[P2] - MHstruct.f_DuoJiP_TableR[P1])*(P_approximation - P1) + MHstruct.f_DuoJiP_TableR[P1];
    }

}

float f_Get_H_approximation(short i16_ViewH) {
  float H_approximation;

  if (i16_ViewH < 0) {
    i16_ViewH = 0;
  }

  H_approximation = i16_ViewH * 3 / MHstruct.f_SizeOfViewH;

  return H_approximation;
}

float f_Get_E_approximation(short i16_E, float f_E_Size) {
  float E_approximation;

  if (i16_E < 0) {
    i16_E = -i16_E;
  }

  E_approximation = i16_E * 2 / f_E_Size;

  return E_approximation;
}


void InitMH(void) 
{
    MHstruct = MHstructFastCar;
    MHstruct.f_SizeOfViewE = 30; //有效偏差
    MHstruct.f_SizeOfViewH = 40; //有效可视距离
  }


int stree_cnt=0;
void Stree_PID_Date(float offset)
{
  
    stree_erro = offset;  //计算当前误差
    
    DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,stree_erro);

    if( ImageStatus.straight_acc_flag==1 && ImageStatus.Road_type != LeftCirque && ImageStatus.Road_type != RightCirque && ImageStatus.Road_type != Ramp)   //直道
    {
      ImageStatus.MU_P=ZD_P; 
      stree_KD=ZD_D;
      // beep.SetGpioValue(1);
    }   
    // else if (ImageStatus.Road_type == LeftCirque)   //左圆
    // {
    //   ImageStatus.MU_P = ZY_P ;
    //   stree_KD = ZY_D ;
    // }  
    // else if (ImageStatus.Road_type == RightCirque)  //右圆
    // {
    //   ImageStatus.MU_P = YY_P ;
    //   stree_KD = YY_D ;
    // }  
    else    //弯道
    {
        ImageStatus.MU_P = WD_P;
      stree_KD=WD_D;
      // beep.SetGpioValue(0);
     
    }       
    // ImageStatus.MU_P=4;
    stree_pid_out=ImageStatus.MU_P*stree_erro+stree_KD*(stree_erro-stree_last_erro);
    stree_last_erro=stree_erro;

    stree_out=stree_mid-stree_pid_out;

    // pwm_set_duty(SERVO_MOTOR1_PWM,4200);
    Stree_Duty(stree_out);
      
    // printf("erro:%.2f   ",stree_erro);
    // printf("out:%d  ",stree_out);
    // printf("P:%.2f\n",ImageStatus.MU_P);
}


