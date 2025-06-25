#include "board.h"
#include "stdio.h"
#include "string.h"
#include "Cal_error_yaw.h"

extern float Yaw;  // �ⲿ����ĵ�ǰƫ���Ǳ���(ȫ��)

/**
 * ���㸡��������ֵ
 * @param value ����ֵ
 * @return ����ֵ
 */
float myfabs(float value) {
    if (value < 0) {
        return -value;
    } else {
        return value;
    }
}

/**
 * ��������ƫ����֮���������(����-180�㵽180���ѭ������)
 * @param Target Ŀ��ƫ����(��)
 * @param Now ��ǰƫ����(��)
 * @return ��̽Ƕ����(��)����ֵ��ʾ��Ҫ��ת����ֵ��ʾ��Ҫ��ת
 */
float Yaw_error(float Target, float Now) {
    static float error;  // ��̬�������������
    Target = -Target;
	Now = -Now;
    // ���1��Ŀ��Ƕ����Ұ�ƽ��(0��~180��)
    if (Target >= 0) {
        // �����1����ǰ�Ƕ������ƽ��(-180��~0��)
        if (Now <= 0) {
            // ���㵱ǰ�Ƕȵľ���ֵ
            float nowAbs = myfabs(Now);
            
            // �ж���˳ʱ����ת������ʱ����ת����
            if (nowAbs < (180 - Target)) {
                // ��ʱ����ת����
                error = nowAbs + Target;
            } else {
                // ˳ʱ����ת����
                error = -(180 - Target) - (180 - nowAbs);
            }
        } 
        // �����2����ǰ�Ƕ����Ұ�ƽ��(0��~180��)
        else if (Now > 0) {
            error = Target - Now;  // ֱ�Ӽ����ֵ
        }
    } 
    // ���2��Ŀ��Ƕ������ƽ��(-180��~0��)
    else if (Target < 0) {
        // �����1����ǰ�Ƕ����Ұ�ƽ��(0��~180��)
        if (Now > 0) {
            // �ж���˳ʱ����ת������ʱ����ת����
            if (Now > Target + 180) {
                // ��ʱ����ת����
                error = (180 - Now) + (180 - myfabs(Target));
            } else if (Now < Target + 180) {
                // ˳ʱ����ת����
                error = -(myfabs(Target) + Now);
            }
        } 
        // �����2����ǰ�Ƕ������ƽ��(-180��~0��)
        else if (Now < 0) {
            error = -(myfabs(Target) - myfabs(Now));  // ͬ����ǶȲ�
        }
    }
    
    return error;  // ���ؼ���õ�����̽Ƕ����
}