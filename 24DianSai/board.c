#include "board.h"
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"
#include "board.h"


#define RE_0_BUFF_LEN_MAX	128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;



void board_init(void)
{
	SYSCFG_DL_init();

	NVIC_ClearPendingIRQ(UART_DM_IMU_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_DM_IMU_INST_INT_IRQN);
}


//����δ�ʱ��ʵ�ֵľ�ȷus��ʱ
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // ������Ҫ��ʱ���� = �ӳ�΢���� * ÿ΢���ʱ����
    ticks = __us * (32000000 / 1000000);

    // ��ȡ��ǰ��SysTickֵ
    told = SysTick->VAL;

    while (1)
    {
        // �ظ�ˢ�»�ȡ��ǰ��SysTickֵ
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // ����ﵽ����Ҫ��ʱ���������˳�ѭ��
            if (tcnt >= ticks)
                break;
        }
    }
}
//����δ�ʱ��ʵ�ֵľ�ȷms��ʱ
void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

////���ڷ��͵����ַ�
//void uart0_send_char(char ch)
//{
//	//������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
//	while( DL_UART_isBusy(UART_MD_IMU_INST) == true );
//	//���͵����ַ�
//	DL_UART_Main_transmitData(UART_MD_IMU_INST, ch);

//}
////���ڷ����ַ���
//void uart0_send_string(char* str)
//{
//	//��ǰ�ַ�����ַ���ڽ�β ���� �ַ����׵�ַ��Ϊ��
//	while(*str!=0&&str!=0)
//	{
//		//�����ַ����׵�ַ�е��ַ��������ڷ������֮���׵�ַ����
//		uart0_send_char(*str++);
//	}
//}


#if !defined(__MICROLIB)
//��ʹ��΢��Ļ�����Ҫ�������ĺ���
#if (__ARMCLIB_VERSION <= 6000000)
//�����������AC5  �Ͷ�����������ṹ��
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
	x = x;
}
#endif


//printf�����ض���

//int fputc(int c, FILE* stream)
//{
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST, c);
//    return c;
//}

//int fputs(const char* restrict s, FILE* restrict stream)
//{
//    uint16_t i, len;
//    len = strlen(s);
//    for(i=0; i<len; i++)
//    {
//        DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST, s[i]);
//    }
//    return len;
//}

int puts(const char *_ptr)
{
    int count = fputs(_ptr, stdout);
    count += fputs("\n", stdout);
    return count;
}




////������ձ���
//uint8_t RollL, RollH, PitchL, PitchH, YawL, YawH, VL, VH, SUM;

//// ���ڽ���״̬��ʶ
//#define WAIT_HEADER1 0
//#define WAIT_HEADER2 1
//#define RECEIVE_DATA 2

//uint8_t RxState = WAIT_HEADER1;
//uint8_t receivedData[9];
//uint8_t dataIndex = 0;



////������ƫ������������
//void Serial_MD_IMU_Zero_Yaw(void){
//   DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XFF);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XAA);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X69);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X88);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XB5);
//	delay_ms(100);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XFF);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XAA);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X01);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X04);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X00);
//	delay_ms(100);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XFF);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XAA);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X00);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X00);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X00);
//	
//}


////������x,y����������
//void Serial_MD_IMU_XY_Yaw(void){
//   DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XFF);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XAA);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X69);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X88);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XB5);
//	delay_ms(100);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XFF);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XAA);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X01);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X08);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X00);
//	delay_ms(100);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XFF);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0XAA);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X00);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X00);
//	DL_UART_Main_transmitDataBlocking(UART_MD_IMU_INST,0X00);
//	
//}


////���ڵ��жϷ�����
//void UART_MD_IMU_INST_IRQHandler(void)
//{
//	 uint8_t uartdata = DL_UART_Main_receiveData(UART_MD_IMU_INST); // ����һ��uint8_t����
//		//LED_Toggle();
//    switch (RxState) {
//    case WAIT_HEADER1:
//        if (uartdata == 0x55) {
//            RxState = WAIT_HEADER2;
//        }
//        break;
//    case WAIT_HEADER2:
//        if (uartdata == 0x53) {
//            RxState = RECEIVE_DATA;
//            dataIndex = 0;
//        } else {
//            RxState = WAIT_HEADER1; // ������������ĵڶ���ͷ������״̬
//        }
//        break;
//    case RECEIVE_DATA:
//        receivedData[dataIndex++] = uartdata;
//        if (dataIndex == 9) {
//            // ���ݽ�����ϣ����������ı���
//            RollL = receivedData[0];
//            RollH = receivedData[1];
//            PitchL = receivedData[2];
//            PitchH = receivedData[3];
//            YawL = receivedData[4];
//            YawH = receivedData[5];
//            VL = receivedData[6];
//            VH = receivedData[7];
//            SUM = receivedData[8];

//            // У��SUM�Ƿ���ȷ
//            uint8_t calculatedSum = 0x55 + 0x53 + RollH + RollL + PitchH + PitchL + YawH + YawL + VH + VL;
//            if (calculatedSum == SUM) {
//                // У��ɹ������Խ��к�������
//                if((float)(((uint16_t)RollH << 8) | RollL)/32768*180>180){
//                    Roll = (float)(((uint16_t)RollH << 8) | RollL)/32768*180 - 360;
//                }else{
//                    Roll = (float)(((uint16_t)RollH << 8) | RollL)/32768*180;
//                }

//                if((float)(((uint16_t)PitchH << 8) | PitchL)/32768*180>180){
//                    Pitch = (float)(((uint16_t)PitchH << 8) | PitchL)/32768*180 - 360;
//                }else{
//                    Pitch = (float)(((uint16_t)PitchH << 8) | PitchL)/32768*180;
//                }

//                if((float)(((uint16_t)YawH << 8) | YawL)/32768*180 >180){
//                    Yaw = (float)(((uint16_t)YawH << 8) | YawL)/32768*180 - 360;
//                }else{
//                    Yaw = (float)(((uint16_t)YawH << 8) | YawL)/32768*180;
//                }
//                //LED_Toggle();
//                
//            } else {
//                // У��ʧ�ܣ��������
//							
//            }
//						//LED_Toggle();
//            RxState = WAIT_HEADER1; // ����״̬�Եȴ���һ�����ݰ�
//        }
//        break;
//    }
//		NVIC_ClearPendingIRQ(UART_MD_IMU_INST_INT_IRQN); //UART
//	/*
//	uint8_t receivedData = 0;
//	
//	//��������˴����ж�
//	switch( DL_UART_getPendingInterrupt(UART_MD_IMU_INST) )
//	{
//		case DL_UART_IIDX_RX://����ǽ����ж�
//			//LED_Toggle();
//			// ���շ��͹��������ݱ���
//			receivedData = DL_UART_Main_receiveData(UART_MD_IMU_INST);

//			// ��黺�����Ƿ�����
//			if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
//			{
//				recv0_buff[recv0_length++] = receivedData;

//				// ������������ٷ��ͳ�ȥ������ش�����ע�͵�
//				uart0_send_char(receivedData);
//			}
//			else
//			{
//				recv0_length = 0;
//			}

//			// ��ǽ��ձ�־
//			recv0_flag = 1;
//		
//			break;
//		
//		default://�����Ĵ����ж�
//			break;
//	}*/
//}

/* IMU����֡���� */
#define IMU_FRAME_LENGTH    19      // ֡����
#define IMU_HEADER_1        0x55    // ֡ͷ1
#define IMU_HEADER_2        0xAA    // ֡ͷ2
#define IMU_TAIL            0x0A    // ֡β

/* ���ջ�������״̬���� */
uint8_t imuBuffer[IMU_FRAME_LENGTH];  // �洢IMU����֡
uint8_t bufferIndex = 0;              // ����������
bool frameReady = false;              // ֡������ɱ�־
float Yaw = 0.0f;

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

void UART_DM_IMU_INST_IRQHandler(void){
	// �ȴ�����һ���ֽ�
        uint8_t byte = DL_UART_receiveDataBlocking(UART_DM_IMU_INST);
		
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
                    Yaw = IMU_ExtractYaw(imuBuffer);
                    frameReady = true;
                }
                
                // ���û�����
                bufferIndex = 0;
            }
        }
}