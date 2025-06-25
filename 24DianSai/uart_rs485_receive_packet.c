/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

/*
 * Configure the internal loopback mode.
 * Note that data received on the UART RXD IO pin will be ignored when
 * loopback is enabled.
 */
#define ENABLE_LOOPBACK_MODE true

/*
 * Number of bytes for UART packet size
 * The packet will be transmitted by the UART.
 * This example uses FIFOs with polling, and the maximum FIFO size is 4.
 * Refer to interrupt examples to handle larger packets.
 */
#define UART_PACKET_SIZE (100)

/* Data received from UART */

/*
uint8_t rxPacket[UART_PACKET_SIZE];

int main(void)
{
    SYSCFG_DL_init();


    for (uint8_t i = 0; i < UART_PACKET_SIZE; i++) {
        rxPacket[i] = DL_UART_receiveDataBlocking(UART_0_INST);
    }


    while (1) {
        DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
        delay_cycles(5000000);
    }
}
*/

#include "ti_msp_dl_config.h"

/* IMU数据帧定义 */
#define IMU_FRAME_LENGTH    100      // 帧长度
#define IMU_HEADER_1        0x55    // 帧头1
#define IMU_HEADER_2        0xAA    // 帧头2
#define IMU_TAIL            0x0A    // 帧尾

/* 接收缓冲区和状态变量 */
uint8_t imuBuffer[IMU_FRAME_LENGTH];  // 存储IMU数据帧
uint8_t bufferIndex = 0;              // 缓冲区索引
bool frameReady = false;              // 帧接收完成标志
float currentYaw = 0.0f;              // 当前Yaw值

/* 从IMU帧中提取Yaw值 */
float IMU_ExtractYaw(const uint8_t* frame) {
    // Yaw值位于DATA[12]-DATA[15]，4字节浮点数(小端格式)
    uint32_t yawInt = 0;
    yawInt |= frame[12];           // L1
    yawInt |= (frame[13] << 8);    // L2
    yawInt |= (frame[14] << 16);   // H1
    yawInt |= (frame[15] << 24);   // H2
    
    // 将整数转换为浮点数
    return *(float*)&yawInt;
}

int main(void)
{
    SYSCFG_DL_init();
    
    // 主循环
    while (1) {
        // 等待接收一个字节
        uint8_t byte = DL_UART_receiveDataBlocking(UART_0_INST);
        
		
		
        // 帧同步和数据收集
        if (bufferIndex == 0) {
            // 等待帧头1
            if (byte == IMU_HEADER_1) {
                imuBuffer[bufferIndex++] = byte;
            }
        } else if (bufferIndex == 1) {
            // 等待帧头2
            if (byte == IMU_HEADER_2) {
                imuBuffer[bufferIndex++] = byte;
            } else {
                // 同步失败，重置
                bufferIndex = 0;
            }
        } else {
            // 收集数据
            imuBuffer[bufferIndex++] = byte;
            
            // 检查是否接收完整帧
            if (bufferIndex >= IMU_FRAME_LENGTH) {
                // 验证帧尾
                if (imuBuffer[IMU_FRAME_LENGTH - 1] == IMU_TAIL) {
                    // 提取Yaw值
                    currentYaw = IMU_ExtractYaw(imuBuffer);
                    frameReady = true;
                }
                
                // 重置缓冲区
                bufferIndex = 0;
            }
        }
        

    }
}  


