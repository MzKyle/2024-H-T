#include "motor.h"
// 速度环PID控制器参数，Kp为比例系数，Ki为积分系数，Kd未使用
float Velcity_Kp=1.0,  Velcity_Ki=0.5,  Velcity_Kd; 

/**
 * 电机A速度环PID控制器 - 增量式PI控制
 * @param TargetVelocity 目标速度值
 * @param CurrentVelocity 当前速度值
 * @return 计算得到的电机A控制量(PWM值)
 */
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  // 速度偏差 = 目标速度 - 当前速度
    static int ControlVelocityA, Last_biasA; // 控制量和上一次的偏差值(静态变量)
    
    Bias = TargetVelocity - CurrentVelocity; // 计算速度偏差
    
    // 增量式PI控制器:
    // ControlVelocityA += Ki*(本次偏差-上次偏差) + Kp*本次偏差
    // Ki项提供积分作用，使系统能够消除静态误差
    // Kp项提供比例作用，产生与偏差成比例的控制响应
    ControlVelocityA += Velcity_Ki*(Bias-Last_biasA) + Velcity_Kp*Bias;
    
    Last_biasA = Bias;  // 保存本次偏差用于下次计算
    
    // 输出限幅保护，防止PWM值过大损坏电机或超出系统范围
    if(ControlVelocityA > 3600) ControlVelocityA = 3600;
    else if(ControlVelocityA < -3600) ControlVelocityA = -3600;
    
    return ControlVelocityA;  // 返回计算后的控制量
}

/**
 * 电机B速度环PID控制器 - 增量式PI控制
 * @param TargetVelocity 目标速度值
 * @param CurrentVelocity 当前速度值
 * @return 计算得到的电机B控制量(PWM值)
 */
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  // 速度偏差
    static int ControlVelocityB, Last_biasB; // 控制量和上一次的偏差值
    
    Bias = TargetVelocity - CurrentVelocity; // 计算速度偏差
    
    // 增量式PI控制器(与电机A相同)
    ControlVelocityB += Velcity_Ki*(Bias-Last_biasB) + Velcity_Kp*Bias;
    
    Last_biasB = Bias;  // 保存本次偏差
    
    // 输出限幅保护
    if(ControlVelocityB > 3600) ControlVelocityB = 3600;
    else if(ControlVelocityB < -3600) ControlVelocityB = -3600;
    
    return ControlVelocityB;  // 返回控制量
}

/**
 * 设置左右电机的PWM输出值
 * @param pwma 电机A的PWM控制值(可为负值，表示反转)
 * @param pwmb 电机B的PWM控制值(可为负值，表示反转)
 */
void Set_PWM(int pwma, int pwmb)
{
    // 设置电机A的PWM值，使用绝对值作为占空比
    DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(pwma), GPIO_PWM_0_C0_IDX);
    
    // 设置电机B的PWM值，使用绝对值作为占空比
    DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(pwmb), GPIO_PWM_0_C1_IDX);
}