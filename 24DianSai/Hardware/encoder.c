#include "encoder.h"
uint32_t gpio_interrup;  // �洢GPIO�ж�״̬�ı���

/**
 * �������жϴ����� - ���������A�ͱ�����B�������ź�
 * �ú�������GROUP1�жϷ���ʱ�Զ�����
 */
void GROUP1_IRQHandler(void)
{
    // ��ȡ���洢���б��������ŵ��ж�״̬
    gpio_interrup = DL_GPIO_getEnabledInterruptStatus(GPIOA, 
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | 
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
    
    //======================== ������A���� ========================
    // ��������A��E1A�����Ƿ񴥷��ж�
    if((gpio_interrup & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
    {
        // ���ݱ�����B��(E1B)�ĵ�ƽ�ж���ת����
        if(!DL_GPIO_readPins(GPIOA, ENCODERA_E1B_PIN))
        {
            // E1BΪ�͵�ƽ: ��ʱ����ת������ֵ��1
            Get_Encoder_countA--;
        }
        else
        {
            // E1BΪ�ߵ�ƽ: ˳ʱ����ת������ֵ��1
            Get_Encoder_countA++;
        }
    }
    // ��������A��E1B�����Ƿ񴥷��ж�
    else if((gpio_interrup & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
    {
        // ���ݱ�����A��(E1A)�ĵ�ƽ�ж���ת����
        if(!DL_GPIO_readPins(GPIOA, ENCODERA_E1A_PIN))
        {
            // E1AΪ�͵�ƽ: ˳ʱ����ת������ֵ��1
            Get_Encoder_countA++;
        }
        else
        {
            // E1AΪ�ߵ�ƽ: ��ʱ����ת������ֵ��1
            Get_Encoder_countA--;
        }
    }
    
    //======================== ������B���� ========================
    // ��������B��E2A�����Ƿ񴥷��ж�
    if((gpio_interrup & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
    {
        // ���ݱ�����B��(E2B)�ĵ�ƽ�ж���ת����
        if(!DL_GPIO_readPins(GPIOA, ENCODERB_E2B_PIN))
        {
            // E2BΪ�͵�ƽ: ��ʱ����ת������ֵ��1
            Get_Encoder_countB--;
        }
        else
        {
            // E2BΪ�ߵ�ƽ: ˳ʱ����ת������ֵ��1
            Get_Encoder_countB++;
        }
    }
    // ��������B��E2B�����Ƿ񴥷��ж�
    else if((gpio_interrup & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
    {
        // ���ݱ�����A��(E2A)�ĵ�ƽ�ж���ת����
        if(!DL_GPIO_readPins(GPIOA, ENCODERB_E2A_PIN))
        {
            // E2AΪ�͵�ƽ: ˳ʱ����ת������ֵ��1
            Get_Encoder_countB++;
        }
        else
        {
            // E2AΪ�ߵ�ƽ: ��ʱ����ת������ֵ��1
            Get_Encoder_countB--;
        }
    }
    
    // ������б��������ŵ��жϱ�־λ
    DL_GPIO_clearInterruptStatus(GPIOA, 
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | 
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}