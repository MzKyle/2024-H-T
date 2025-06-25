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

/* IMU����֡���� */
#define IMU_FRAME_LENGTH    100      // ֡����
#define IMU_HEADER_1        0x55    // ֡ͷ1
#define IMU_HEADER_2        0xAA    // ֡ͷ2
#define IMU_TAIL            0x0A    // ֡β

/* ���ջ�������״̬���� */
uint8_t imuBuffer[IMU_FRAME_LENGTH];  // �洢IMU����֡
uint8_t bufferIndex = 0;              // ����������
bool frameReady = false;              // ֡������ɱ�־
float currentYaw = 0.0f;              // ��ǰYawֵ

/* ��IMU֡����ȡYawֵ */
float IMU_ExtractYaw(const uint8_t* frame) {
    // Yawֵλ��DATA[12]-DATA[15]��4�ֽڸ�����(С�˸�ʽ)
    uint32_t yawInt = 0;
    yawInt |= frame[12];           // L1
    yawInt |= (frame[13] << 8);    // L2
    yawInt |= (frame[14] << 16);   // H1
    yawInt |= (frame[15] << 24);   // H2
    
    // ������ת��Ϊ������
    return *(float*)&yawInt;
}

int main(void)
{
    SYSCFG_DL_init();
    
    // ��ѭ��
    while (1) {
        // �ȴ�����һ���ֽ�
        uint8_t byte = DL_UART_receiveDataBlocking(UART_0_INST);
        
		
		
        // ֡ͬ���������ռ�
        if (bufferIndex == 0) {
            // �ȴ�֡ͷ1
            if (byte == IMU_HEADER_1) {
                imuBuffer[bufferIndex++] = byte;
            }
        } else if (bufferIndex == 1) {
            // �ȴ�֡ͷ2
            if (byte == IMU_HEADER_2) {
                imuBuffer[bufferIndex++] = byte;
            } else {
                // ͬ��ʧ�ܣ�����
                bufferIndex = 0;
            }
        } else {
            // �ռ�����
            imuBuffer[bufferIndex++] = byte;
            
            // ����Ƿ��������֡
            if (bufferIndex >= IMU_FRAME_LENGTH) {
                // ��֤֡β
                if (imuBuffer[IMU_FRAME_LENGTH - 1] == IMU_TAIL) {
                    // ��ȡYawֵ
                    currentYaw = IMU_ExtractYaw(imuBuffer);
                    frameReady = true;
                }
                
                // ���û�����
                bufferIndex = 0;
            }
        }
        

    }
}  


