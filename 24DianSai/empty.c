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

extern tPid PID_Link;               //定义一个循迹结构体
int deviation = 0;

int PWML=0;					
int PWML_Base=1000;

int PWMR=0;				
int PWMR_Base=1000;
int PWML_MAX = 1200;
int PWMR_MAX = 1200;
int TrackFlag=1;		//定义循迹标志位，为0停止循迹，为1开始循迹
int read_flag = 1; 
extern uint8_t sensor_arr[8];
extern int error;
int track_err = 0;
extern int Yaw_err;
extern int Yaw_err2;
float imu_err = 0;
float imu_err2 = 0;


#define PI 3.14159265

int One_Wheel_len = 204;		 //mm  一轮长度
int One_Wheel_Mai = 730;   	//一圈脉冲数
float Wheel_count = 0.2794; //mm 一个脉冲数转长度
float carL_dis = 0;
float carR_dis = 0;


float dis_Q1 = 980;
float distance = 940;	//mm 
float distance2 = 1000;	//mm 
float dis1=905;
float dis2=950;


int move_flag = 1;
int RunMode = 0;       // 运行模式

extern float Yaw;
int imu_read_flag = 1;

int time_flag = 0;
int time_count = 0;



int L_B_flag = 0;
int quan_num =0;
int Q4_flag = 0;
int readKey_flag = 1;

/**按键Flag**/
int key_count = 0;

int key2_count = 0;

int Task_Num = 1; //要求1、2、3、4

int Task_Flag = 0; //0无效

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
	

	DL_GPIO_setPins(RGB_GREEN_PORT,RGB_GREEN_PIN_0_PIN); // 绿灯 1灭
	DL_GPIO_clearPins(GPIOB, RGB_Red_PIN_26_PIN);// RGB红灯 1亮
	DL_GPIO_clearPins(GPIOA, BEEF_PIN_27_PIN );//BEEF 1响
	
	delay_ms(1000);
	Set_PWM(0,0);
	PID_init(&PID_Link,0,70,0,200,1000,-1000);   //循迹PID初始化
	
test = DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN);	
	
   while (1) 
   {
     //OLED_ShowString(8,0*16,number,OLED_8X16);
     //OLED_Update();
	   
		 test = DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN);
	   
		 if(Task_Flag == 1) //任务1
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
								imu_err = Yaw_error(0,Yaw);  //0与IMU的Yaw的差
							}			
							
							deviation = (int32_t)PID_realize(&PID_Link,imu_err/1.8); //获取PID的计算结果
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
	 
if(1) //(Task_Flag == 2)  //任务2
{
    readKey_flag = 0;  // 禁止按键读取，防止在任务执行过程中因按键操作干扰任务
    switch(2)  //(RunMode)
    {
        case 0:
        {
            // 初始化编码器计数值和车辆行驶距离
            Get_Encoder_countA = 0;
            Get_Encoder_countB = 0;
            carL_dis = 0;
            carR_dis = 0;
            // 进入下一阶段
            RunMode++;
            break;
        }
        case 1:
        {
            // 计算左右车轮的行驶距离
            carR_dis = Wheel_count * -Get_Encoder_countB;
            carL_dis = Wheel_count * Get_Encoder_countA;
            // 判断车辆是否行驶到指定距离
            if((fabsf(carL_dis) >= distance) && (fabsf(carR_dis) >= distance))
            {
                // 达到指定距离，停止移动
                move_flag = 0;
            }

            if(move_flag == 1)
            {
                // 如果允许移动
                if(imu_read_flag == 1)
                {
                  
                    imu_err = Yaw_error(0, Yaw);
                }
                // 通过PID算法计算控制偏差
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 1.8);
                // 根据偏差调整左右电机的PWM值
                PWML = PWML_Base - deviation;
                PWMR = PWMR_Base + deviation;

                // 限制PWM值在有效范围内
                if(PWML > PWML_MAX)
                    PWML = PWML_MAX;
                else if(PWML < 0)
                    PWML = 0;

                if(PWMR > PWMR_MAX)
                    PWMR = PWMR_MAX;
                else if(PWMR < 0)
                    PWMR = 0;
                // 设置电机的PWM值
                Set_PWM(PWML, PWMR);
            }
            else
            {
                // 如果不允许移动，停止电机
                Set_PWM(0, 0);
                // 开启循迹功能
                TrackFlag = 1;
                // 启动定时器标志
                time_flag = 1;
                // 进入下一阶段
                RunMode++;
            }
            break;
        }
        case 2:
        {
            if(TrackFlag == 1)  // 开启循迹功能
            {
                if(read_flag == 1)
                {
                    // 读取传感器数据
                    sensor_read();
                    // 获取循迹的偏差
                    track_err = track_error();
                }

                if(track_err < 30000)
                {
                    // 通过PID算法根据循迹偏差计算控制偏差
                    deviation = (int32_t)PID_realize(&PID_Link, track_err);
                    // 根据偏差调整左右电机的PWM值
                    PWML = PWML_Base - deviation;
                    PWMR = PWMR_Base + deviation;

                    // 限制PWM值在有效范围内
                    if(PWML > PWML_MAX)
                        PWML = PWML_MAX;
                    else if(PWML < 0)
                        PWML = 0;

                    if(PWMR > PWMR_MAX)
                        PWMR = PWMR_MAX;
                    else if(PWMR < 0)
                        PWMR = 0;
                    // 设置电机的PWM值
                    Set_PWM(PWML, PWMR);
                }
                else
                {
                    // 循迹偏差过大，停止电机
                    Set_PWM(0, 0);
                    // 关闭循迹功能
                    TrackFlag = 0;
                    // 此处注释掉的代码可能用于重新初始化PID参数
                    // PID_init(&PID_Link,0,70,0,400,800,-800);
                }
            }
            else
            {
                // 允许移动
                move_flag = 1;
                // 启动定时器标志
                time_flag = 1;
                // 进入下一阶段
                RunMode++;
            }
            break;
        }
        case 3:
        {
            // 初始化编码器计数值和车辆行驶距离
            Get_Encoder_countA = 0;
            Get_Encoder_countB = 0;
            carL_dis = 0;
            carR_dis = 0;
            // 进入下一阶段
            RunMode++;
            break;
        }
        case 4:
        {
            // 计算左右车轮的行驶距离
            carR_dis = Wheel_count * -Get_Encoder_countB;
            carL_dis = Wheel_count * Get_Encoder_countA;
            // 判断车辆是否行驶到指定距离（此处未实际使用该判断）
            if((fabsf(carL_dis) >= distance) && (fabsf(carR_dis) >= distance))
            {
                // move_flag = 0;
            }

            // 读取传感器数据
            sensor_read();
            // 检查传感器数据，若有传感器值为0则停止移动
            for(int i = 0; i < 8; i++)
            {
                if(sensor_arr[i] == 0)
                {
                    move_flag = 0;
                }
            }

            if(move_flag == 1)
            {
                // 如果允许移动
                if(imu_read_flag == 1)
                {
                    // 读取IMU数据，计算偏航角误差（目标偏航角为 -178）
                    imu_err = -Yaw_error(-178, Yaw);
                }
                // 通过PID算法计算控制偏差
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 2.8);
                // 根据偏差调整左右电机的PWM值
                PWML = PWML_Base - deviation;
                PWMR = PWMR_Base + deviation;

                // 限制PWM值在有效范围内
                if(PWML > PWML_MAX)
                    PWML = PWML_MAX;
                else if(PWML < 0)
                    PWML = 0;

                if(PWMR > PWMR_MAX)
                    PWMR = PWMR_MAX;
                else if(PWMR < 0)
                    PWMR = 0;
                // 设置电机的PWM值（此处左右电机PWM值交换）
                Set_PWM(PWMR, PWML);
            }
            else
            {
                // 如果不允许移动，停止电机
                Set_PWM(0, 0);
                // 开启循迹功能
                TrackFlag = 1;
                // 启动定时器标志
                time_flag = 1;
                // 进入下一阶段
                RunMode++;
            }
            break;
        }
        case 5:
        {
            if(TrackFlag == 1)  // 开启循迹功能
            {
                if(read_flag == 1)
                {
                    // 读取传感器数据
                    sensor_read();
                    // 获取循迹的偏差
                    track_err = track_error();
                }

                if(track_err < 30000)
                {
                    // 通过PID算法根据循迹偏差计算控制偏差
                    deviation = (int32_t)PID_realize(&PID_Link, track_err);
                    // 根据偏差调整左右电机的PWM值
                    PWML = PWML_Base - deviation;
                    PWMR = PWMR_Base + deviation;

                    // 限制PWM值在有效范围内
                    if(PWML > PWML_MAX)
                        PWML = PWML_MAX;
                    else if(PWML < 0)
                        PWML = 0;

                    if(PWMR > PWMR_MAX)
                        PWMR = PWMR_MAX;
                    else if(PWMR < 0)
                        PWMR = 0;
                    // 设置电机的PWM值
                    Set_PWM(PWML, PWMR);
                }
                else
                {
                    // 循迹偏差过大，停止电机
                    Set_PWM(0, 0);
                    // 关闭循迹功能
                    TrackFlag = 0;
                }
            }
            else
            {
                // 允许移动
                move_flag = 1;
                // 启动定时器标志
                time_flag = 1;
                // 进入下一阶段
                RunMode++;
                // 任务完成，重置任务标志
                Task_Flag = 0;
                // 允许按键读取
                readKey_flag = 1;
            }
            break;
        }
    }
}
		
		
if (Task_Flag == 3 || Task_Flag == 4) {
    readKey_flag = 0; // 禁止按键输入，避免任务中断
    switch (RunMode) {
        case 0: // 阶段0：初始化（第一阶段直线移动前）
            Get_Encoder_countA = 0; 
            Get_Encoder_countB = 0; 
            carL_dis = 0, carR_dis = 0; 
            RunMode++; // 进入下一阶段（直线移动）
            break;

        case 1: // 阶段1：第一段直线移动（目标角度-53°）
            // 计算行驶距离（编码器脉冲转距离）
            carR_dis = Wheel_count * -Get_Encoder_countB;
            carL_dis = Wheel_count * Get_Encoder_countA; 

            if ((fabsf(carL_dis) >= dis1) && (fabsf(carR_dis) >= dis1)) {
                move_flag = 0; 
            }
            if (move_flag == 1) { // 未到达，继续移动
                if (imu_read_flag == 1) {
                    // 根据quan_num选择目标角度（首次为-53°，后续可能调整dis1）
                    if (quan_num == 0) {
                        imu_err = Yaw_error(-53, Yaw); // 计算偏航角误差（目标-53°）
                    } else {
                        dis1 = 905; // 调整目标距离
                        imu_err = Yaw_error(-53, Yaw);
                    }
                }
                // PID计算偏差，调整电机PWM
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 3.2);
                PWML = PWML_Base - deviation; // 左电机PWM（基准值-偏差）
                PWMR = PWMR_Base + deviation; // 右电机PWM（基准值+偏差）
                // 限制PWM范围（0到最大值）
                PWML = (PWML > PWML_MAX) ? PWML_MAX : (PWML < 0 ? 0 : PWML);
                PWMR = (PWMR > PWMR_MAX) ? PWMR_MAX : (PWMR < 0 ? 0 : PWMR);
                Set_PWM(PWML, PWMR); // 设置电机PWM
            } else { // 到达距离或停止移动
                Set_PWM(0, 0); // 停止电机
                TrackFlag = 0; // 关闭循迹功能
                RunMode++; // 进入下一阶段（循迹准备）
            }
            break;

        case 2: // 阶段2：循迹阶段（TrackFlag控制是否启用循迹）
            if (TrackFlag == 0) { // 未开启循迹，调整方向（目标0°）
                imu_err = Yaw_error(0, Yaw); // 计算偏航角误差（目标0°）
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 3.6);
                PWML = PWML_Base - deviation;
                PWMR = PWMR_Base + deviation;
                Set_PWM(PWML, PWMR); // 调整方向，使车辆对准赛道
                // 检测灰度传感器，若检测到赛道（传感器值为0），开启循迹
                sensor_read();
                for (int i = 0; i < 8; i++) {
                    if (sensor_arr[i] == 0) {
                        TrackFlag = 1; // 开启循迹功能
                        time_flag = 1; 
                    }
                }
            } else if (TrackFlag == 1) { // 开启循迹，根据赛道偏差调整
                sensor_read(); // 读取灰度传感器数据
                track_err = track_error(); // 计算循迹偏差（偏离赛道中心的距离）
                if (track_err < 30000) { // 有效偏差（非全白赛道）
                    deviation = (int32_t)PID_realize(&PID_Link, track_err);
                    PWML = PWML_Base - deviation;
                    PWMR = PWMR_Base + deviation;
                    Set_PWM(PWML, PWMR); // 按偏差调整电机，保持循迹
                } else { // 偏差过大（全白赛道，可能为终点）
                    Set_PWM(0, 0); // 停止电机
                    TrackFlag = 2; // 标记循迹结束
                    move_flag = 1; // 允许移动（可能进入下一阶段）
                    time_flag = 1;
                    RunMode++; // 进入下一阶段（第二段直线移动准备）
                }
            }
            break;

        case 3: // 阶段3：中间初始化（切换阶段时重置编码器）
            Get_Encoder_countA = 0;
            Get_Encoder_countB = 0;
            carL_dis = 0, carR_dis = 0;
            RunMode++; // 进入下一阶段（第二段直线移动）
            break;

        case 4: // 阶段4：第二段直线移动（目标角度-134°）
            carR_dis = Wheel_count * -Get_Encoder_countB;
            carL_dis = Wheel_count * Get_Encoder_countA;
            if ((fabsf(carL_dis) >= dis2) && (fabsf(carR_dis) >= dis2)) {
                move_flag = 0; // 到达目标距离（dis2）
            }
            if (move_flag == 1) { // 未到达，继续移动
                if (imu_read_flag == 1) {
                    // 根据quan_num调整目标角度和距离（任务4循环时使用）
                    if (quan_num == 0) {
                        imu_err = Yaw_error(-134, Yaw); // 目标角度-134°
                    } else {
                        imu_err = Yaw_error(-134, Yaw);
                        dis1 = 950;
                    }
                }
                // PID计算并设置PWM（逻辑同阶段1）
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 3.5);
                PWML = PWML_Base - deviation;
                PWMR = PWMR_Base + deviation;
                Set_PWM(PWML, PWMR);
            } else { // 到达距离或停止移动
                Set_PWM(0, 0);
                TrackFlag = 0; // 关闭循迹
                RunMode++; // 进入下一阶段（最终循迹或任务结束）
            }
            break;

        case 5: // 阶段5：最终处理（任务3结束/任务4循环）
            if (TrackFlag == 0) { // 调整方向（目标-180°，可能为返程）
                imu_err = Yaw_error(-180, Yaw);
                deviation = (int32_t)PID_realize(&PID_Link, imu_err / 3.5);
                Set_PWM(PWML, PWMR); // 调整方向
                // 检测赛道，触发循迹
                sensor_read();
                for (int i = 0; i < 8; i++) {
                    if (sensor_arr[i] == 0) {
                        TrackFlag = 1; // 再次开启循迹
                        time_flag = 1;
                    }
                }
            } else if (TrackFlag == 1) { // 循迹控制（逻辑同阶段2）
                sensor_read();
                track_err = track_error();
                if (track_err < 30000) {
                    deviation = (int32_t)PID_realize(&PID_Link, track_err);
                    Set_PWM(PWML, PWMR);
                } else { // 任务3结束/任务4循环
                    if (Task_Flag == 3) { // 任务3：直接结束
                        Set_PWM(0, 0);
                        TrackFlag = 2;
                        move_flag = 1;
                        Task_Flag = 0; // 重置任务标志
                        readKey_flag = 1; // 允许按键输入
                    } else if (Task_Flag == 4) { // 任务4：循环执行
                        TrackFlag = 2;
                        move_flag = 1;
                        quan_num++; // 循环计数
                        if (quan_num >= 4) { // 循环4次后结束
                            quan_num = 0;
                            Set_PWM(0, 0);
                            Task_Flag = 0;
                            readKey_flag = 1;
                        } else {
                            RunMode = 0; // 重置阶段，重新开始任务
                        }
                    }
                }
            }
            break;
    }
}

	
	}
}


//按键中间是A，左边是B
void TIMER_0_INST_IRQHandler(void)
{
    // 检查定时器 TIMER_0_INST 是否有挂起的中断
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        // 检查是否是定时器的特定中断（DL_TIMER_IIDX_ZERO）
        if(DL_TIMER_IIDX_ZERO)
        {
            // 读取编码器的值，由于两个电机安装相反，encoderB_cnt 取反
            encoderA_cnt = Get_Encoder_countA;
            encoderB_cnt = -Get_Encoder_countB;

            // 按下两次S2，开始执行任务

            if(DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN)==0) 
            {
                // 按键按下，计数器 key2_count 加 1
                key2_count++;
                if(key2_count >= 4)
                {
                    // 计数器达到 4 时，重置计数器
                    key2_count = 0;	
                    // 再次确认按键按下且允许读取按键
                    if(DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN)==0 && readKey_flag == 1)
                    {
                        // 等待按键释放
                        while(DL_GPIO_readPins(KEY_START_PORT,KEY_START_PIN_1_PIN)==0);
                        // 根据当前任务编号设置任务标志
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

            //S3选择任务
            if(DL_GPIO_readPins(KEY_MODE_PORT,KEY_MODE_PIN_21_PIN)==0) 
            {
                // 按键按下，计数器 key_count 加 1
                key_count++;
                if(key_count >= 4)
                {
                    // 计数器达到 4 时，重置计数器
                    key_count = 0;
                    // 再次确认按键按下
                    if(DL_GPIO_readPins(KEY_MODE_PORT,KEY_MODE_PIN_21_PIN)==0 )
                    {
                        // 等待按键释放
                        while(DL_GPIO_readPins(KEY_MODE_PORT,KEY_MODE_PIN_21_PIN)==0);
                        // 任务编号加 1
                        Task_Num ++;
                        if(Task_Num >4)
                        {
                            // 任务编号超过 4 时，重置为 1
                            Task_Num = 1;
                        }
                        // 将任务编号转换为字符串存储在 number 数组中
                        sprintf(number,"%d",Task_Num);
                    }
                }
            }

            // 如果时间标志 time_flag 为 1
            if(time_flag == 1)
            {		
                // 时间计数器 time_count 加 1
                time_count++;
                // 点亮 RGB 红灯
                DL_GPIO_setPins(GPIOB, RGB_Red_PIN_26_PIN);
                // 熄灭绿灯
                DL_GPIO_clearPins(RGB_GREEN_PORT,RGB_GREEN_PIN_0_PIN); 
                // 开启蜂鸣器
                DL_GPIO_setPins(GPIOA, BEEF_PIN_27_PIN );
                if(time_count >= 15) 
                {
                    // 时间计数器达到 15 时，重置计数器和时间标志
                    time_count=0;
                    time_flag = 0;
                    // 熄灭绿灯
                    DL_GPIO_setPins(RGB_GREEN_PORT,RGB_GREEN_PIN_0_PIN); 
                    // 熄灭 RGB 红灯
                    DL_GPIO_clearPins(GPIOB, RGB_Red_PIN_26_PIN);
                    // 关闭蜂鸣器
                    DL_GPIO_clearPins(GPIOA, BEEF_PIN_27_PIN );
                }
            }
        }
    }
}