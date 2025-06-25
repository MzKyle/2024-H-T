#include "encoder.h"
uint32_t gpio_interrup;  // 存储GPIO中断状态的变量

/**
 * 编码器中断处理函数 - 处理编码器A和编码器B的正交信号
 * 该函数会在GROUP1中断发生时自动调用
 */
void GROUP1_IRQHandler(void)
{
    // 读取并存储所有编码器引脚的中断状态
    gpio_interrup = DL_GPIO_getEnabledInterruptStatus(GPIOA, 
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | 
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
    
    //======================== 编码器A处理 ========================
    // 检测编码器A的E1A引脚是否触发中断
    if((gpio_interrup & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
    {
        // 根据编码器B相(E1B)的电平判断旋转方向
        if(!DL_GPIO_readPins(GPIOA, ENCODERA_E1B_PIN))
        {
            // E1B为低电平: 逆时针旋转，计数值减1
            Get_Encoder_countA--;
        }
        else
        {
            // E1B为高电平: 顺时针旋转，计数值加1
            Get_Encoder_countA++;
        }
    }
    // 检测编码器A的E1B引脚是否触发中断
    else if((gpio_interrup & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
    {
        // 根据编码器A相(E1A)的电平判断旋转方向
        if(!DL_GPIO_readPins(GPIOA, ENCODERA_E1A_PIN))
        {
            // E1A为低电平: 顺时针旋转，计数值加1
            Get_Encoder_countA++;
        }
        else
        {
            // E1A为高电平: 逆时针旋转，计数值减1
            Get_Encoder_countA--;
        }
    }
    
    //======================== 编码器B处理 ========================
    // 检测编码器B的E2A引脚是否触发中断
    if((gpio_interrup & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
    {
        // 根据编码器B相(E2B)的电平判断旋转方向
        if(!DL_GPIO_readPins(GPIOA, ENCODERB_E2B_PIN))
        {
            // E2B为低电平: 逆时针旋转，计数值减1
            Get_Encoder_countB--;
        }
        else
        {
            // E2B为高电平: 顺时针旋转，计数值加1
            Get_Encoder_countB++;
        }
    }
    // 检测编码器B的E2B引脚是否触发中断
    else if((gpio_interrup & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
    {
        // 根据编码器A相(E2A)的电平判断旋转方向
        if(!DL_GPIO_readPins(GPIOA, ENCODERB_E2A_PIN))
        {
            // E2A为低电平: 顺时针旋转，计数值加1
            Get_Encoder_countB++;
        }
        else
        {
            // E2A为高电平: 逆时针旋转，计数值减1
            Get_Encoder_countB--;
        }
    }
    
    // 清除所有编码器引脚的中断标志位
    DL_GPIO_clearInterruptStatus(GPIOA, 
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | 
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}