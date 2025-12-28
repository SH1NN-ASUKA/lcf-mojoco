#ifndef __MOTOR_H__
#define __MOTOR_H__

//右电机    
#define MOTOR1_DIR   "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM   "/dev/zf_device_pwm_motor_1"

//左电机
#define MOTOR2_DIR   "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM   "/dev/zf_device_pwm_motor_2"

//编码器
#define ENCODER_1           "/dev/zf_encoder_1"
#define ENCODER_2           "/dev/zf_encoder_2"

extern int16 encoder_left;
extern int16 encoder_right;
extern int16_t speed_out_r;
extern int16_t speed_out_l;
extern int16_t Speed_Goal_l;
extern int16_t Speed_Goal_r;

//记录编码器
extern int licheng_l;
extern int licheng_r;
extern int lichengji;
extern int Record_encoder;
extern int16 Speed_Goal;
void Get_encoder(void);
void motor_Init(void);
void Speed_decision(void);
void motor_Duty_R(int duty);
void motor_Duty_L(int duty);
int16_t encoder_num_L(void);
int16_t encoder_num_R(void);
void Speed_BiHuan(void);
void Car_Speed_Contrl(void);
void Car_distance(void);

#endif