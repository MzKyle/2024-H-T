#include "pid.h"


tPid PID_Link;  // ����һ��ѭ�������õ�PID�ṹ��ʵ��

/**
 * ��ʼ��PID����������
 * @param pid      PID�ṹ��ָ��
 * @param target_val Ŀ��ֵ
 * @param Kp       ����ϵ��
 * @param Ki       ����ϵ��
 * @param Kd       ΢��ϵ��
 * @param MAX      ������ֵ����
 * @param MIN      �����Сֵ����
 */
void PID_init(tPid *pid, float target_val, float Kp, float Ki, float Kd, float MAX, float MIN)
{
    // ����PID����������
    pid->target_val = target_val;  // �趨Ŀ��ֵ
    pid->Kp = Kp;                  // ����ϵ��
    pid->Ki = Ki;                  // ����ϵ��
    pid->Kd = Kd;                  // ΢��ϵ��
    pid->MAX = MAX;                // �������
    pid->MIN = MIN;                // �������

    // ��ʼ���������ֵ
    pid->err = 0;                  // ��ǰ�������
    pid->err_last = 0;             // �ϴ��������
    pid->err_pre = 0;              // ���ϴ��������(������δʹ��)
    pid->err_sum = 0;              // ����ۻ�������
    pid->actual_val = 0;           // ʵ�����ֵ����
}

/**
 * λ��ʽPID������ʵ��
 * @param pid        PID�ṹ��ָ��
 * @param actual_val ��ǰʵ��ֵ
 * @return           �����Ŀ��������
 */
float PID_realize(tPid *pid, float actual_val)
{
    // ���µ�ǰʵ��ֵ
    pid->actual_val = actual_val;
    
    // ���㵱ǰ���: Ŀ��ֵ - ʵ��ֵ
    pid->err = pid->target_val - pid->actual_val;
    
    // �ۻ�������(���ڻ�����)
    pid->err_sum += pid->err;
    
    // λ��ʽPID���㹫ʽ:
    // Output = Kp*e(k) + Ki*��e(k) + Kd*[e(k)-e(k-1)]
    pid->actual_val = pid->Kp * pid->err          // ������
                    + pid->Ki * pid->err_sum      // ������
                    + pid->Kd * (pid->err - pid->err_last);  // ΢����
    
    // ���浱ǰ���������һ�μ���
    pid->err_last = pid->err;
    
    // ����޷���������ֹ������������ȫ��Χ
    if (pid->actual_val > pid->MAX) {
        pid->actual_val = pid->MAX;
    } else if (pid->actual_val < pid->MIN) {
        pid->actual_val = pid->MIN;
    }

    // ���ؼ����Ŀ�����
    return pid->actual_val;
}