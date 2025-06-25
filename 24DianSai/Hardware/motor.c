#include "motor.h"
// �ٶȻ�PID������������KpΪ����ϵ����KiΪ����ϵ����Kdδʹ��
float Velcity_Kp=1.0,  Velcity_Ki=0.5,  Velcity_Kd; 

/**
 * ���A�ٶȻ�PID������ - ����ʽPI����
 * @param TargetVelocity Ŀ���ٶ�ֵ
 * @param CurrentVelocity ��ǰ�ٶ�ֵ
 * @return ����õ��ĵ��A������(PWMֵ)
 */
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  // �ٶ�ƫ�� = Ŀ���ٶ� - ��ǰ�ٶ�
    static int ControlVelocityA, Last_biasA; // ����������һ�ε�ƫ��ֵ(��̬����)
    
    Bias = TargetVelocity - CurrentVelocity; // �����ٶ�ƫ��
    
    // ����ʽPI������:
    // ControlVelocityA += Ki*(����ƫ��-�ϴ�ƫ��) + Kp*����ƫ��
    // Ki���ṩ�������ã�ʹϵͳ�ܹ�������̬���
    // Kp���ṩ�������ã�������ƫ��ɱ����Ŀ�����Ӧ
    ControlVelocityA += Velcity_Ki*(Bias-Last_biasA) + Velcity_Kp*Bias;
    
    Last_biasA = Bias;  // ���汾��ƫ�������´μ���
    
    // ����޷���������ֹPWMֵ�����𻵵���򳬳�ϵͳ��Χ
    if(ControlVelocityA > 3600) ControlVelocityA = 3600;
    else if(ControlVelocityA < -3600) ControlVelocityA = -3600;
    
    return ControlVelocityA;  // ���ؼ����Ŀ�����
}

/**
 * ���B�ٶȻ�PID������ - ����ʽPI����
 * @param TargetVelocity Ŀ���ٶ�ֵ
 * @param CurrentVelocity ��ǰ�ٶ�ֵ
 * @return ����õ��ĵ��B������(PWMֵ)
 */
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  // �ٶ�ƫ��
    static int ControlVelocityB, Last_biasB; // ����������һ�ε�ƫ��ֵ
    
    Bias = TargetVelocity - CurrentVelocity; // �����ٶ�ƫ��
    
    // ����ʽPI������(����A��ͬ)
    ControlVelocityB += Velcity_Ki*(Bias-Last_biasB) + Velcity_Kp*Bias;
    
    Last_biasB = Bias;  // ���汾��ƫ��
    
    // ����޷�����
    if(ControlVelocityB > 3600) ControlVelocityB = 3600;
    else if(ControlVelocityB < -3600) ControlVelocityB = -3600;
    
    return ControlVelocityB;  // ���ؿ�����
}

/**
 * �������ҵ����PWM���ֵ
 * @param pwma ���A��PWM����ֵ(��Ϊ��ֵ����ʾ��ת)
 * @param pwmb ���B��PWM����ֵ(��Ϊ��ֵ����ʾ��ת)
 */
void Set_PWM(int pwma, int pwmb)
{
    // ���õ��A��PWMֵ��ʹ�þ���ֵ��Ϊռ�ձ�
    DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(pwma), GPIO_PWM_0_C0_IDX);
    
    // ���õ��B��PWMֵ��ʹ�þ���ֵ��Ϊռ�ձ�
    DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(pwmb), GPIO_PWM_0_C1_IDX);
}