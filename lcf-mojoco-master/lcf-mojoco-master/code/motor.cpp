#include "zf_common_headfile.h"

//      模块管脚            单片机管脚
//      MOTOR1_DIR          GPIO73
//      MOTOR1_PWM          GPIO65
//      GND                 GND   
//      MOTOR2_DIR          GPIO72
//      MOTOR2_PWM          GPIO66
//      GND                 GND
//      接线端子 +          电池正极
//      接线端子 -          电池负极



// 接入方向编码器连接 1
//      模块管脚            单片机管脚
//      LSB                 ENCODER_1_LSB 宏定义的引脚 默认 GPIO64
//      DIR                 ENCODER_1_DIR 宏定义的引脚 默认 GPIO51
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
// 
// 接入方向编码器连接 2
//      模块管脚            单片机管脚
//      LSB                 ENCODER_2_LSB 宏定义的引脚 默认 GPIO67
//      DIR                 ENCODER_2_DIR 宏定义的引脚 默认 GPIO50
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//



// 接入电调 主板上对应有电调的接口 务必注意不要插反 <红色接NC> <黑色接地> <黄色/橙色/棕色/白色...其它彩色的那根是信号>
//      模块管脚            单片机管脚
//      PWM                 GPIO88
//      PWM                 GPIO89
//      GND                 电调电源 GND 连通 核心板电源地 GND

//电机
#define MOTOR1_DIR   "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM   "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR   "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM   "/dev/zf_device_pwm_motor_2"

// 编码器
#define ENCODER_1           "/dev/zf_encoder_1"
#define ENCODER_2           "/dev/zf_encoder_2"

// 无刷
struct pwm_info pwm_1_info;
struct pwm_info pwm_2_info;
#define PWM_1           "/dev/zf_device_pwm_esc_1"
#define PWM_2           "/dev/zf_device_pwm_esc_2"

// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (motor_1_pwm_info.duty_max)       
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。 
#define MOTOR2_PWM_DUTY_MAX    (motor_2_pwm_info.duty_max)   

struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;
int licheng_l = 0;
int licheng_r = 0;
int lichengji = 0;
//记录编码器
int Record_encoder=0;

//实际编码器速度
int16 encoder_left; 
int16 encoder_right;   

// 速度
int16 Speed_Goal=0;
int ZD_Speed_Goal;  //直道速度
int WD_Speed_Goal;  //弯道速度
int ZY_Speed_Goal;  //左圆速度
int YY_Speed_Goal;  //右圆速度

//左闭环变量
int16_t Speed_Goal_l=0; //期望速度
int16_t speed_P_l, speed_D_l;
int16_t speed_out_l;
int speed_erro_L, speed_last_erro_L;

float speed_I_l;
float speed_I_r;
//右闭环变量
int16_t Speed_Goal_r=0; //期望速度
int16_t speed_P_r, speed_D_r;
int16_t speed_out_r;
int speed_erro_R, speed_last_erro_R;

float Left_Speed_Co = 1.550, Right_Speed_Co = 1.550;
float CS=0.44;//0.44//0.3


void motor_Init(void)
{
    // 获取电机PWM设备信息
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);

    // 获取无刷PWM设备信息
    pwm_get_dev_info(PWM_1, &pwm_1_info);
    pwm_get_dev_info(PWM_2, &pwm_2_info);
}

void motor_Duty_R(int duty)
{
 
    if (duty <= 0)  //反转
    {
        gpio_set_level(MOTOR1_DIR, 0);        // DIR输出高电平
        pwm_set_duty(MOTOR1_PWM, -duty);       // 计算占空比
    }
    else    
    {
        gpio_set_level(MOTOR1_DIR, 1);        // DIR输出高电平
        pwm_set_duty(MOTOR1_PWM, duty);       // 计算占空比
    }

}

void motor_Duty_L(int duty)
{
 
    if (duty <= 0)  //反转
    {
        gpio_set_level(MOTOR2_DIR, 0);        // DIR输出高电平
        pwm_set_duty(MOTOR2_PWM, -duty);       // 计算占空比
    }
    else    
    {
        gpio_set_level(MOTOR2_DIR, 1);        // DIR输出高电平
        pwm_set_duty(MOTOR2_PWM, duty);       // 计算占空比
    }

}
void Get_encoder(void)
{
    if (ImageFlag.RoadBlock_Flag != 0)
    {
        lichengji += (encoder_num_L() + encoder_num_R()) / 2;
        licheng_l += encoder_num_L();
        licheng_r += encoder_num_R();
    }
    else {
        lichengji = 0;
        licheng_l = 0;
        licheng_r = 0;
    }
    if (lichengji > 1000000) lichengji = 0;
    if (licheng_l > 1000000) licheng_l = 0;
    if (licheng_r > 1000000) licheng_r = 0;
}

//闭环参数初始化
void Speed_decision(void)
{
    
    speed_P_l = 3;
    speed_D_l = 20;
    speed_I_l = 1;

    speed_P_r = 3;
    speed_D_r = 20;
    speed_I_r = 1;
    // Speed_Goal = 150;
    WD_Speed_Goal=400;   //弯道速    77   530
    ZD_Speed_Goal=400;     //直道88    85

    ZY_Speed_Goal=400;  //左圆环速度
    YY_Speed_Goal=400;  //右圆环速度
 
}

void Speed_PID_L(void)
{
    speed_erro_L = Speed_Goal_l - encoder_left;
    speed_out_l += speed_P_l * speed_erro_L + speed_D_l * (speed_erro_L - speed_last_erro_L);
    speed_last_erro_L = speed_erro_L;
    if (speed_out_l > 7000)
    {
        speed_out_l = 7000;
    }
    if (speed_out_l < -7000)
    {
        speed_out_l = -7000;
    }
}


void Speed_PID_R(void)
{
    speed_erro_R = Speed_Goal_r - encoder_right;
    speed_out_r += speed_P_r * speed_erro_R + speed_D_r * (speed_erro_R - speed_last_erro_R);
    speed_last_erro_R = speed_erro_R;
    if (speed_out_r > 7000)
    {
        speed_out_r = 7000;
    }
    if (speed_out_r < -7000)
    {
        speed_out_r = -7000;
    }
}

void Speed_PID_Incremental_L(void)
{
    speed_erro = Speed_Goal_l - encoder_left;

    // 带积分限幅的增量PID
    float delta_out = speed_P_l * (speed_erro - speed_last_erro)
        + speed_I_l * speed_erro
        + speed_D_l * (speed_erro - 2 * speed_last_erro + speed_last_last_erro);

    // 积分抗饱和处理
    if (fabsf(delta_out) > 500) {
        delta_out = (delta_out > 0) ? 500 : -500;
    }

    speed_out_l += (int)delta_out;

    // 更新历史误差
    speed_last_last_erro = speed_last_erro;
    speed_last_erro = speed_erro;

    // 输出限幅
    speed_out_l = (speed_out > 7000) ? 7000 :
        (speed_out < -7000) ? -7000 : speed_out;
}

void Speed_PID_Incremental_R(void)
{
    speed_erro = Speed_Goal_r - encoder_right;

    // 带积分限幅的增量PID
    float delta_out = speed_P_r * (speed_erro - speed_last_erro)
        + speed_I_r * speed_erro
        + speed_D_r * (speed_erro - 2 * speed_last_erro + speed_last_last_erro);

    // 积分抗饱和处理
    if (fabsf(delta_out) > 500) {
        delta_out = (delta_out > 0) ? 500 : -500;
    }

    speed_out_r += (int)delta_out;

    // 更新历史误差
    speed_last_last_erro = speed_last_erro;
    speed_last_erro = speed_erro;

    // 输出限幅
    speed_out_r = (speed_out > 7000) ? 7000 :
        (speed_out < -7000) ? -7000 : speed_out;
}
/*************************************************************************/
//编码器
//正转是返回正数
int16_t encoder_num_L(void)
{
    int16 Temp;
    Temp = encoder_get_count(ENCODER_1);

    return Temp;
}

int16_t encoder_num_R(void)
{
    int16 Temp;
    Temp = encoder_get_count(ENCODER_2);

    return -Temp;
}
/*********************************************************************************/


//变速控制
void Car_Speed_Contrl(void)
{
        //直线加速
    if(ImageStatus.straight_acc==1 && ImageStatus.Road_type != LeftCirque && ImageStatus.Road_type != RightCirque && ImageStatus.Road_type != Ramp)
    {
        Speed_Goal = ZD_Speed_Goal;
        // FuYa_Speed_Goal=120000;
        // beep.SetGpioValue(1);
    }
    else if(ImageStatus.Road_type == LeftCirque)    // 左圆环
    {
        Speed_Goal = ZY_Speed_Goal;
        // FuYa_Speed_Goal=140000;
        // beep.SetGpioValue(1);
    }
    else if(ImageStatus.Road_type == RightCirque)   //右圆环
    {
        Speed_Goal = YY_Speed_Goal;
    }
    else
    {
        Speed_Goal=WD_Speed_Goal;
    }
    // FuYa_Speed_Goal=140000;

         
    if(ImageFlag.Zebra_Flag == 1)  //停车
    {
        Speed_Goal=0;
        
    }
}


void Speed_BiHuan(void)
{

    encoder_left  = encoder_num_L();
    encoder_right = encoder_num_R();
    Car_Speed_Contrl();     //变速控制 
    Speed_Goal_l = Speed_Goal;
    Speed_Goal_r = Speed_Goal;
    Speed_PID_L();
    Speed_PID_R();
    motor_Duty_R(speed_out_r);
    motor_Duty_L(speed_out_l);

    // motor_Duty_L(-1000);
    // motor_Duty_R(1000);

    // pwm_set_duty(PWM_1, duty);
    // pwm_set_duty(PWM_2, 600);
    // 打印PWM频率和duty最大值
    // printf("pwm 1 freq = %d Hz\r\n", pwm_1_info.freq);
    // printf("pwm 1 duty_max = %d\r\n", pwm_1_info.duty_max);

    // printf("pwm 2 freq = %d Hz\r\n", pwm_2_info.freq);
    // printf("pwm 2 duty_max = %d\r\n", pwm_2_info.duty_max);


    // printf("l:%d     ", encoder_left);
    // printf("r:%d\n", encoder_right);
    // printf("    ");
    
}