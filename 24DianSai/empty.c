#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "board.h"
#include "motor.h"
#include "Sensor.h"
#include "PID.h"
#include "Cal_error_yaw.h"


int32_t Get_Encoder_countA,encoderA_cnt,PWMA,Get_Encoder_countB,encoderB_cnt,PWMB;
uint8_t Key_Num = 0;

extern tPid PID_Link;               //����һ��ѭ���ṹ��
int deviation = 0;

int PWML=0;					
int PWML_Base=1000;

int PWMR=0;				
int PWMR_Base=1000;
int PWML_MAX = 1200;
int PWMR_MAX = 1200;
int TrackFlag=1;		//����ѭ����־λ��Ϊ0ֹͣѭ����Ϊ1��ʼѭ��
int read_flag = 1; 
extern uint8_t sensor_arr[8];
extern int error;
int track_err = 0;
extern int Yaw_err;
extern int Yaw_err2;
float imu_err = 0;
float imu_err2 = 0;


#define PI 3.14159265

int One_Wheel_len = 204;		 //mm  һ�ֳ���
int One_Wheel_Mai = 730;   	//һȦ������
float Wheel_count = 0.2794; //mm һ��������ת����
float carL_dis = 0;
float carR_dis = 0;


float dis_Q1 = 980;
float distance = 940;	//mm 
float distance2 = 1000;	//mm 
float dis1=905;
float dis2=950;


int move_flag = 1;
int RunMode = 0;       // ����ģʽ

extern float Yaw;
int imu_read_flag = 1;

int time_flag = 0;
int time_count = 0;



int L_B_flag = 0;
int quan_num =0;
int Q4_flag = 0;
int readKey_flag = 1;

/**����Flag**/
int key_count = 0;

int key2_count = 0;

int Task_Num = 1; //Ҫ��1��2��3��4

int Task_Flag = 0; //0��Ч

char number[]="1";

int test;
	
int main(void)
{
	SYSCFG_DL_init();
	DL_Timer_startCounter(PWM_0_INST); //PWM

	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);//TIMER_INT
	NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(UART_DM_IMU_INST_INT_IRQN);
		
	//	NVIC_EnableIRQ(I2C_MPU6050_INST_INT_IRQN); 
	//	NVIC_ClearPendingIRQ(I2C_MPU6050_INST_INT_IRQN);
	

	DL_GPIO_setPins(RGB_GREEN_PORT,RGB_GREEN_PIN_0_PIN); // �̵� 1��
	DL_GPIO_clearPins(GPIOB, RGB_Red_PIN_26_PIN);// RGB��� 1��
	DL_GPIO_clearPins(GPIOA, BEEF_PIN_27_PIN );//BEEF 1��
	
	delay_ms(1000);
	Set_PWM(0,0);
	PID_init(&PID_Link,0,70,0,200,1000,-1000);   //ѭ��PID��ʼ��
	
test = DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN);	
	
   while (1) 
   {
     //OLED_ShowString(8,0*16,number,OLED_8X16);
     //OLED_Update();
	   
		 test = DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN);
	   
		 if(Task_Flag == 1) //����1
		 {	
			 switch(RunMode)
			 {
				 case 0:
				 {
						Get_Encoder_countA=0;
						Get_Encoder_countB=0;
						carL_dis=0,carR_dis=0;
						RunMode++;
						break;
				 }
				 case 1:
				 {
					readKey_flag = 0;
					carR_dis = Wheel_count * -Get_Encoder_countB;
					carL_dis = Wheel_count * Get_Encoder_countA;
					if((fabsf(carL_dis) >= distance) && (fabsf(carR_dis) >= distance))
					{
						move_flag = 0;
					}
					
					if(move_flag ==1)
					{
						 if(imu_read_flag == 1)
							{
								imu_err = Yaw_error(0,Yaw);  //0��IMU��Yaw�Ĳ�
							}			
							
							deviation = (int32_t)PID_realize(&PID_Link,imu_err/1.8); //��ȡPID�ļ�����
							PWML = PWML_Base - deviation;
							PWMR = PWMR_Base + deviation ;
										
										
							if(PWML > PWML_MAX)
									PWML = PWML_MAX;
							 else if(PWML < 0)
									PWML=0;
								 
							 if(PWMR > PWMR_MAX)
									PWMR = PWMR_MAX;
							 else if(PWMR < 0)
									PWMR=0;
							Set_PWM(PWML,PWMR);
					}
					else
					{
							Set_PWM(0,0);
							time_flag = 1;
							Task_Flag = 0;
							readKey_flag = 1;
					}	
					break;
				}
			}				
		 }
	 
if(1) //(Task_Flag == 2)  //����2
{
    readKey_flag = 0;  // ��ֹ������ȡ����ֹ������ִ�й������򰴼�������������
    switch(2)  //(RunMode)
    {
        case 0:
        {
            // ��ʼ������������ֵ�ͳ�����ʻ����
            Get_Encoder_countA = 0;
            Get_Encoder_countB = 0;
            carL_dis = 0;
            carR_dis = 0;
            // ������һ�׶�
            RunMode++;
            break;
        }
        case 1:
        {
            // �������ҳ��ֵ���ʻ����
            carR_dis = Wheel_count * -Get_Encoder_countB;
            carL_dis = Wheel_count * Get_Encoder_countA;
            // �жϳ����Ƿ���ʻ��ָ������
            if((fabsf(carL_dis) >= distance) && (fabsf(carR_dis) >= distance))
            {
                // �ﵽָ�����룬ֹͣ�ƶ�
                move_flag = 0;
            }

            if(move_flag == 1)
            {
                // ��������ƶ�
                if(imu_read_flag == 1)
                {
                  
                    imu_err = Yaw_error(0, Yaw);
                }
                // ͨ��PID�㷨�������ƫ��
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 1.8);
                // ����ƫ��������ҵ����PWMֵ
                PWML = PWML_Base - deviation;
                PWMR = PWMR_Base + deviation;

                // ����PWMֵ����Ч��Χ��
                if(PWML > PWML_MAX)
                    PWML = PWML_MAX;
                else if(PWML < 0)
                    PWML = 0;

                if(PWMR > PWMR_MAX)
                    PWMR = PWMR_MAX;
                else if(PWMR < 0)
                    PWMR = 0;
                // ���õ����PWMֵ
                Set_PWM(PWML, PWMR);
            }
            else
            {
                // ����������ƶ���ֹͣ���
                Set_PWM(0, 0);
                // ����ѭ������
                TrackFlag = 1;
                // ������ʱ����־
                time_flag = 1;
                // ������һ�׶�
                RunMode++;
            }
            break;
        }
        case 2:
        {
            if(TrackFlag == 1)  // ����ѭ������
            {
                if(read_flag == 1)
                {
                    // ��ȡ����������
                    sensor_read();
                    // ��ȡѭ����ƫ��
                    track_err = track_error();
                }

                if(track_err < 30000)
                {
                    // ͨ��PID�㷨����ѭ��ƫ��������ƫ��
                    deviation = (int32_t)PID_realize(&PID_Link, track_err);
                    // ����ƫ��������ҵ����PWMֵ
                    PWML = PWML_Base - deviation;
                    PWMR = PWMR_Base + deviation;

                    // ����PWMֵ����Ч��Χ��
                    if(PWML > PWML_MAX)
                        PWML = PWML_MAX;
                    else if(PWML < 0)
                        PWML = 0;

                    if(PWMR > PWMR_MAX)
                        PWMR = PWMR_MAX;
                    else if(PWMR < 0)
                        PWMR = 0;
                    // ���õ����PWMֵ
                    Set_PWM(PWML, PWMR);
                }
                else
                {
                    // ѭ��ƫ�����ֹͣ���
                    Set_PWM(0, 0);
                    // �ر�ѭ������
                    TrackFlag = 0;
                    // �˴�ע�͵��Ĵ�������������³�ʼ��PID����
                    // PID_init(&PID_Link,0,70,0,400,800,-800);
                }
            }
            else
            {
                // �����ƶ�
                move_flag = 1;
                // ������ʱ����־
                time_flag = 1;
                // ������һ�׶�
                RunMode++;
            }
            break;
        }
        case 3:
        {
            // ��ʼ������������ֵ�ͳ�����ʻ����
            Get_Encoder_countA = 0;
            Get_Encoder_countB = 0;
            carL_dis = 0;
            carR_dis = 0;
            // ������һ�׶�
            RunMode++;
            break;
        }
        case 4:
        {
            // �������ҳ��ֵ���ʻ����
            carR_dis = Wheel_count * -Get_Encoder_countB;
            carL_dis = Wheel_count * Get_Encoder_countA;
            // �жϳ����Ƿ���ʻ��ָ�����루�˴�δʵ��ʹ�ø��жϣ�
            if((fabsf(carL_dis) >= distance) && (fabsf(carR_dis) >= distance))
            {
                // move_flag = 0;
            }

            // ��ȡ����������
            sensor_read();
            // ��鴫�������ݣ����д�����ֵΪ0��ֹͣ�ƶ�
            for(int i = 0; i < 8; i++)
            {
                if(sensor_arr[i] == 0)
                {
                    move_flag = 0;
                }
            }

            if(move_flag == 1)
            {
                // ��������ƶ�
                if(imu_read_flag == 1)
                {
                    // ��ȡIMU���ݣ�����ƫ������Ŀ��ƫ����Ϊ -178��
                    imu_err = -Yaw_error(-178, Yaw);
                }
                // ͨ��PID�㷨�������ƫ��
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 2.8);
                // ����ƫ��������ҵ����PWMֵ
                PWML = PWML_Base - deviation;
                PWMR = PWMR_Base + deviation;

                // ����PWMֵ����Ч��Χ��
                if(PWML > PWML_MAX)
                    PWML = PWML_MAX;
                else if(PWML < 0)
                    PWML = 0;

                if(PWMR > PWMR_MAX)
                    PWMR = PWMR_MAX;
                else if(PWMR < 0)
                    PWMR = 0;
                // ���õ����PWMֵ���˴����ҵ��PWMֵ������
                Set_PWM(PWMR, PWML);
            }
            else
            {
                // ����������ƶ���ֹͣ���
                Set_PWM(0, 0);
                // ����ѭ������
                TrackFlag = 1;
                // ������ʱ����־
                time_flag = 1;
                // ������һ�׶�
                RunMode++;
            }
            break;
        }
        case 5:
        {
            if(TrackFlag == 1)  // ����ѭ������
            {
                if(read_flag == 1)
                {
                    // ��ȡ����������
                    sensor_read();
                    // ��ȡѭ����ƫ��
                    track_err = track_error();
                }

                if(track_err < 30000)
                {
                    // ͨ��PID�㷨����ѭ��ƫ��������ƫ��
                    deviation = (int32_t)PID_realize(&PID_Link, track_err);
                    // ����ƫ��������ҵ����PWMֵ
                    PWML = PWML_Base - deviation;
                    PWMR = PWMR_Base + deviation;

                    // ����PWMֵ����Ч��Χ��
                    if(PWML > PWML_MAX)
                        PWML = PWML_MAX;
                    else if(PWML < 0)
                        PWML = 0;

                    if(PWMR > PWMR_MAX)
                        PWMR = PWMR_MAX;
                    else if(PWMR < 0)
                        PWMR = 0;
                    // ���õ����PWMֵ
                    Set_PWM(PWML, PWMR);
                }
                else
                {
                    // ѭ��ƫ�����ֹͣ���
                    Set_PWM(0, 0);
                    // �ر�ѭ������
                    TrackFlag = 0;
                }
            }
            else
            {
                // �����ƶ�
                move_flag = 1;
                // ������ʱ����־
                time_flag = 1;
                // ������һ�׶�
                RunMode++;
                // ������ɣ����������־
                Task_Flag = 0;
                // ��������ȡ
                readKey_flag = 1;
            }
            break;
        }
    }
}
		
		
if (Task_Flag == 3 || Task_Flag == 4) {
    readKey_flag = 0; // ��ֹ�������룬���������ж�
    switch (RunMode) {
        case 0: // �׶�0����ʼ������һ�׶�ֱ���ƶ�ǰ��
            Get_Encoder_countA = 0; 
            Get_Encoder_countB = 0; 
            carL_dis = 0, carR_dis = 0; 
            RunMode++; // ������һ�׶Σ�ֱ���ƶ���
            break;

        case 1: // �׶�1����һ��ֱ���ƶ���Ŀ��Ƕ�-53�㣩
            // ������ʻ���루����������ת���룩
            carR_dis = Wheel_count * -Get_Encoder_countB;
            carL_dis = Wheel_count * Get_Encoder_countA; 

            if ((fabsf(carL_dis) >= dis1) && (fabsf(carR_dis) >= dis1)) {
                move_flag = 0; 
            }
            if (move_flag == 1) { // δ��������ƶ�
                if (imu_read_flag == 1) {
                    // ����quan_numѡ��Ŀ��Ƕȣ��״�Ϊ-53�㣬�������ܵ���dis1��
                    if (quan_num == 0) {
                        imu_err = Yaw_error(-53, Yaw); // ����ƫ������Ŀ��-53�㣩
                    } else {
                        dis1 = 905; // ����Ŀ�����
                        imu_err = Yaw_error(-53, Yaw);
                    }
                }
                // PID����ƫ��������PWM
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 3.2);
                PWML = PWML_Base - deviation; // ����PWM����׼ֵ-ƫ�
                PWMR = PWMR_Base + deviation; // �ҵ��PWM����׼ֵ+ƫ�
                // ����PWM��Χ��0�����ֵ��
                PWML = (PWML > PWML_MAX) ? PWML_MAX : (PWML < 0 ? 0 : PWML);
                PWMR = (PWMR > PWMR_MAX) ? PWMR_MAX : (PWMR < 0 ? 0 : PWMR);
                Set_PWM(PWML, PWMR); // ���õ��PWM
            } else { // ��������ֹͣ�ƶ�
                Set_PWM(0, 0); // ֹͣ���
                TrackFlag = 0; // �ر�ѭ������
                RunMode++; // ������һ�׶Σ�ѭ��׼����
            }
            break;

        case 2: // �׶�2��ѭ���׶Σ�TrackFlag�����Ƿ�����ѭ����
            if (TrackFlag == 0) { // δ����ѭ������������Ŀ��0�㣩
                imu_err = Yaw_error(0, Yaw); // ����ƫ������Ŀ��0�㣩
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 3.6);
                PWML = PWML_Base - deviation;
                PWMR = PWMR_Base + deviation;
                Set_PWM(PWML, PWMR); // ��������ʹ������׼����
                // ���Ҷȴ�����������⵽������������ֵΪ0��������ѭ��
                sensor_read();
                for (int i = 0; i < 8; i++) {
                    if (sensor_arr[i] == 0) {
                        TrackFlag = 1; // ����ѭ������
                        time_flag = 1; 
                    }
                }
            } else if (TrackFlag == 1) { // ����ѭ������������ƫ�����
                sensor_read(); // ��ȡ�Ҷȴ���������
                track_err = track_error(); // ����ѭ��ƫ�ƫ���������ĵľ��룩
                if (track_err < 30000) { // ��Чƫ���ȫ��������
                    deviation = (int32_t)PID_realize(&PID_Link, track_err);
                    PWML = PWML_Base - deviation;
                    PWMR = PWMR_Base + deviation;
                    Set_PWM(PWML, PWMR); // ��ƫ��������������ѭ��
                } else { // ƫ�����ȫ������������Ϊ�յ㣩
                    Set_PWM(0, 0); // ֹͣ���
                    TrackFlag = 2; // ���ѭ������
                    move_flag = 1; // �����ƶ������ܽ�����һ�׶Σ�
                    time_flag = 1;
                    RunMode++; // ������һ�׶Σ��ڶ���ֱ���ƶ�׼����
                }
            }
            break;

        case 3: // �׶�3���м��ʼ�����л��׶�ʱ���ñ�������
            Get_Encoder_countA = 0;
            Get_Encoder_countB = 0;
            carL_dis = 0, carR_dis = 0;
            RunMode++; // ������һ�׶Σ��ڶ���ֱ���ƶ���
            break;

        case 4: // �׶�4���ڶ���ֱ���ƶ���Ŀ��Ƕ�-134�㣩
            carR_dis = Wheel_count * -Get_Encoder_countB;
            carL_dis = Wheel_count * Get_Encoder_countA;
            if ((fabsf(carL_dis) >= dis2) && (fabsf(carR_dis) >= dis2)) {
                move_flag = 0; // ����Ŀ����루dis2��
            }
            if (move_flag == 1) { // δ��������ƶ�
                if (imu_read_flag == 1) {
                    // ����quan_num����Ŀ��ǶȺ;��루����4ѭ��ʱʹ�ã�
                    if (quan_num == 0) {
                        imu_err = Yaw_error(-134, Yaw); // Ŀ��Ƕ�-134��
                    } else {
                        imu_err = Yaw_error(-134, Yaw);
                        dis1 = 950;
                    }
                }
                // PID���㲢����PWM���߼�ͬ�׶�1��
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 3.5);
                PWML = PWML_Base - deviation;
                PWMR = PWMR_Base + deviation;
                Set_PWM(PWML, PWMR);
            } else { // ��������ֹͣ�ƶ�
                Set_PWM(0, 0);
                TrackFlag = 0; // �ر�ѭ��
                RunMode++; // ������һ�׶Σ�����ѭ�������������
            }
            break;

        case 5: // �׶�5�����մ�������3����/����4ѭ����
            if (TrackFlag == 0) { // ��������Ŀ��-180�㣬����Ϊ���̣�
                imu_err = Yaw_error(-180, Yaw);
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 3.5);
                Set_PWM(PWML, PWMR); // ��������
                // �������������ѭ��
                sensor_read();
                for (int i = 0; i < 8; i++) {
                    if (sensor_arr[i] == 0) {
                        TrackFlag = 1; // �ٴο���ѭ��
                        time_flag = 1;
                    }
                }
            } else if (TrackFlag == 1) { // ѭ�����ƣ��߼�ͬ�׶�2��
                sensor_read();
                track_err = track_error();
                if (track_err < 30000) {
                    deviation = (int32_t)PID_realize(&PID_Link, track_err);
                    Set_PWM(PWML, PWMR);
                } else { // ����3����/����4ѭ��
                    if (Task_Flag == 3) { // ����3��ֱ�ӽ���
                        Set_PWM(0, 0);
                        TrackFlag = 2;
                        move_flag = 1;
                        Task_Flag = 0; // ���������־
                        readKey_flag = 1; // ����������
                    } else if (Task_Flag == 4) { // ����4��ѭ��ִ��
                        TrackFlag = 2;
                        move_flag = 1;
                        quan_num++; // ѭ������
                        if (quan_num >= 4) { // ѭ��4�κ����
                            quan_num = 0;
                            Set_PWM(0, 0);
                            Task_Flag = 0;
                            readKey_flag = 1;
                        } else {
                            RunMode = 0; // ���ý׶Σ����¿�ʼ����
                        }
                    }
                }
            }
            break;
    }
}

	
	}
}


//�����м���A�������B
void TIMER_0_INST_IRQHandler(void)
{
    // ��鶨ʱ�� TIMER_0_INST �Ƿ��й�����ж�
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        // ����Ƿ��Ƕ�ʱ�����ض��жϣ�DL_TIMER_IIDX_ZERO��
        if(DL_TIMER_IIDX_ZERO)
        {
            // ��ȡ��������ֵ���������������װ�෴��encoderB_cnt ȡ��
            encoderA_cnt = Get_Encoder_countA;
            encoderB_cnt = -Get_Encoder_countB;

            // ��������S2����ʼִ������

            if(DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN)==0) 
            {
                // �������£������� key2_count �� 1
                key2_count++;
                if(key2_count >= 4)
                {
                    // �������ﵽ 4 ʱ�����ü�����
                    key2_count = 0;	
                    // �ٴ�ȷ�ϰ��������������ȡ����
                    if(DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN)==0 && readKey_flag == 1)
                    {
                        // �ȴ������ͷ�
                        while(DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN)==0);
                        // ���ݵ�ǰ���������������־
                        if(Task_Num == 1)
                            Task_Flag = 1;
                        if(Task_Num == 2)
                            Task_Flag = 2;
                        if(Task_Num == 3)
                            Task_Flag = 3;
                        if(Task_Num == 4)
                            Task_Flag = 4;
                    }
                }
            }

            //S3ѡ������
            if(DL_GPIO_readPins(KEY_MODE_PORT,KEY_MODE_PIN_21_PIN)==0) 
            {
                // �������£������� key_count �� 1
                key_count++;
                if(key_count >= 4)
                {
                    // �������ﵽ 4 ʱ�����ü�����
                    key_count = 0;
                    // �ٴ�ȷ�ϰ�������
                    if(DL_GPIO_readPins(KEY_MODE_PORT,KEY_MODE_PIN_21_PIN)==0 )
                    {
                        // �ȴ������ͷ�
                        while(DL_GPIO_readPins(KEY_MODE_PORT,KEY_MODE_PIN_21_PIN)==0);
                        // �����ż� 1
                        Task_Num ++;
                        if(Task_Num >4)
                        {
                            // �����ų��� 4 ʱ������Ϊ 1
                            Task_Num = 1;
                        }
                        // ��������ת��Ϊ�ַ����洢�� number ������
                        sprintf(number,"%d",Task_Num);
                    }
                }
            }

            // ���ʱ���־ time_flag Ϊ 1
            if(time_flag == 1)
            {		
                // ʱ������� time_count �� 1
                time_count++;
                // ���� RGB ���
                DL_GPIO_setPins(GPIOB, RGB_Red_PIN_26_PIN);
                // Ϩ���̵�
                DL_GPIO_clearPins(RGB_GREEN_PORT,RGB_GREEN_PIN_0_PIN); 
                // ����������
                DL_GPIO_setPins(GPIOA, BEEF_PIN_27_PIN );
                if(time_count >= 15) 
                {
                    // ʱ��������ﵽ 15 ʱ�����ü�������ʱ���־
                    time_count=0;
                    time_flag = 0;
                    // Ϩ���̵�
                    DL_GPIO_setPins(RGB_GREEN_PORT,RGB_GREEN_PIN_0_PIN); 
                    // Ϩ�� RGB ���
                    DL_GPIO_clearPins(GPIOB, RGB_Red_PIN_26_PIN);
                    // �رշ�����
                    DL_GPIO_clearPins(GPIOA, BEEF_PIN_27_PIN );
                }
            }
        }
    }
}