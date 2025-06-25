#include "pid.h"


tPid PID_Link;  // 定义一个循迹控制用的PID结构体实例

/**
 * 初始化PID控制器参数
 * @param pid      PID结构体指针
 * @param target_val 目标值
 * @param Kp       比例系数
 * @param Ki       积分系数
 * @param Kd       微分系数
 * @param MAX      输出最大值限制
 * @param MIN      输出最小值限制
 */
void PID_init(tPid *pid, float target_val, float Kp, float Ki, float Kd, float MAX, float MIN)
{
    // 设置PID控制器参数
    pid->target_val = target_val;  // 设定目标值
    pid->Kp = Kp;                  // 比例系数
    pid->Ki = Ki;                  // 积分系数
    pid->Kd = Kd;                  // 微分系数
    pid->MAX = MAX;                // 输出上限
    pid->MIN = MIN;                // 输出下限

    // 初始化误差和输出值
    pid->err = 0;                  // 当前误差清零
    pid->err_last = 0;             // 上次误差清零
    pid->err_pre = 0;              // 上上次误差清零(代码中未使用)
    pid->err_sum = 0;              // 误差累积和清零
    pid->actual_val = 0;           // 实际输出值清零
}

/**
 * 位置式PID控制器实现
 * @param pid        PID结构体指针
 * @param actual_val 当前实际值
 * @return           计算后的控制量输出
 */
float PID_realize(tPid *pid, float actual_val)
{
    // 更新当前实际值
    pid->actual_val = actual_val;
    
    // 计算当前误差: 目标值 - 实际值
    pid->err = pid->target_val - pid->actual_val;
    
    // 累积误差求和(用于积分项)
    pid->err_sum += pid->err;
    
    // 位置式PID计算公式:
    // Output = Kp*e(k) + Ki*∑e(k) + Kd*[e(k)-e(k-1)]
    pid->actual_val = pid->Kp * pid->err          // 比例项
                    + pid->Ki * pid->err_sum      // 积分项
                    + pid->Kd * (pid->err - pid->err_last);  // 微分项
    
    // 保存当前误差用于下一次计算
    pid->err_last = pid->err;
    
    // 输出限幅保护，防止控制量超出安全范围
    if (pid->actual_val > pid->MAX) {
        pid->actual_val = pid->MAX;
    } else if (pid->actual_val < pid->MIN) {
        pid->actual_val = pid->MIN;
    }

    // 返回计算后的控制量
    return pid->actual_val;
}