#ifndef __STREE_H__
#define __STREE_H__

#include "zf_common_headfile.h"

#define H_Min   2
#define stree_mid 1500 
#define stree_left 1700 
#define stree_right 1300

extern float stree_erro;
extern int32_t stree_out;


/*模糊控制部分*/
typedef struct {
//基础模糊表
uint8_t         ui8_Table[4][4];
} MH_Table;


typedef struct {
//舵机P值表L
float         f_DuoJiP_TableL[7];
//舵机P值表R
float         f_DuoJiP_TableR[7];
//舵机模糊表
MH_Table      mt_Duoji[4];
//电机P值表
float         f_DianJiP_Table[7];
//电机I值表
float         f_DianJiI_Table[7];
//电机模糊表
MH_Table      mt_DianJi[4];
//出入弯标志
uint8_t         ui8_IO;
//左右出入弯标志
uint8_t         ui8_IOLR;
//出入弯加减速标志
uint8_t         ui8_IOAS;
//反向可视距离变化范围
float         f_SizeOfViewH;
//中值偏差变化范围
float         f_SizeOfViewE;
//脉冲偏差变化范围
float         f_SizeOfPulseE;
//上次反向可视距离
short         i16_ViewH;
//
} MH;
extern MH MHstructFastCar;
extern MH MHstruct;

float f_Get_E_approximation(short i16_E, float f_E_Size);
float f_Get_H_approximation(short i16_ViewH);
void DuoJi_GetP(float* i32p_P, short i16_ViewH, short i16_ViewE);
void InitMH(void);


void Stree_Init(void);
void Stree_Duty(uint32_t duty);
void Stree_PID_Date(float offset);


#endif // !__STREE_H__
